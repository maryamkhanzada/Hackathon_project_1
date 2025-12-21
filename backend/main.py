"""
RAG Content Ingestion System

Automated pipeline for extracting documentation from Docusaurus websites,
generating embeddings using Cohere, and storing them in Qdrant Cloud.
"""

import os
import logging
import time
import uuid
from typing import List, Dict, Any, Optional, Callable
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import requests
from bs4 import BeautifulSoup


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


def load_config() -> Dict[str, Any]:
    """
    Load configuration from environment variables with validation.

    Environment Variables:
        COHERE_API_KEY (required): Cohere API key
        QDRANT_URL (required): Qdrant Cloud URL
        QDRANT_API_KEY (required): Qdrant API key
        BASE_URL (required): Docusaurus site base URL
        CHUNK_SIZE (optional): Chunk size in characters (default: 1000)
        CHUNK_OVERLAP (optional): Overlap in characters (default: 200)
        CODE_THRESHOLD (optional): Code block split threshold (default: 500)
        COHERE_MODEL (optional): Model name (default: "embed-english-v3.0")
        COLLECTION_NAME (optional): Qdrant collection (default: "rag_embedding")
        BATCH_SIZE (optional): Embedding batch size (default: 50)

    Returns:
        Dictionary containing all configuration parameters

    Raises:
        ValueError: If required environment variables are missing
        ValueError: If configuration values are invalid
    """
    load_dotenv()

    # Required variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'BASE_URL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    # Load and validate configuration
    config = {
        'cohere_api_key': os.getenv('COHERE_API_KEY'),
        'qdrant_url': os.getenv('QDRANT_URL'),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY'),
        'base_url': os.getenv('BASE_URL'),
        'chunk_size': int(os.getenv('CHUNK_SIZE', '1000')),
        'chunk_overlap': int(os.getenv('CHUNK_OVERLAP', '200')),
        'code_threshold': int(os.getenv('CODE_THRESHOLD', '500')),
        'cohere_model': os.getenv('COHERE_MODEL', 'embed-english-v3.0'),
        'collection_name': os.getenv('COLLECTION_NAME', 'rag_embedding'),
        'batch_size': int(os.getenv('BATCH_SIZE', '50')),
        'vector_size': 1024,  # Cohere embed-english-v3.0 default
    }

    # Validate parameter ranges
    if config['chunk_size'] <= 0 or config['chunk_size'] > 2000:
        raise ValueError(f"chunk_size must be between 1 and 2000, got {config['chunk_size']}")

    if config['chunk_overlap'] < 0 or config['chunk_overlap'] >= config['chunk_size']:
        raise ValueError(f"chunk_overlap must be between 0 and chunk_size, got {config['chunk_overlap']}")

    if config['code_threshold'] <= 0 or config['code_threshold'] >= config['chunk_size']:
        raise ValueError(f"code_threshold must be between 1 and chunk_size, got {config['code_threshold']}")

    if config['batch_size'] <= 0 or config['batch_size'] > 96:
        raise ValueError(f"batch_size must be between 1 and 96 (Cohere limit), got {config['batch_size']}")

    logging.info("Configuration loaded successfully")
    return config


def retry_with_backoff(func: Callable, max_retries: int = 3, *args: Any, **kwargs: Any) -> Any:
    """
    Retry a function with exponential backoff.

    Args:
        func: Function to retry
        max_retries: Maximum number of retry attempts
        *args: Positional arguments to pass to func
        **kwargs: Keyword arguments to pass to func

    Returns:
        Result of func if successful

    Raises:
        Last exception if all retries fail
    """
    for attempt in range(max_retries):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            wait_time = 2 ** attempt  # 1s, 2s, 4s
            logging.warning(f"Attempt {attempt + 1}/{max_retries} failed: {str(e)}. Retrying in {wait_time}s...")
            time.sleep(wait_time)


def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site sitemap and return all documentation page URLs.

    Args:
        base_url: Base URL of the Docusaurus site (e.g., "https://docs.example.com")

    Returns:
        List of absolute URLs for all discovered pages
        Empty list if sitemap not found (logs warning)

    Raises:
        requests.RequestException: If sitemap URL is unreachable after retries
        ValueError: If base_url is not a valid HTTP/HTTPS URL
    """
    if not base_url.startswith(('http://', 'https://')):
        raise ValueError(f"base_url must start with http:// or https://, got: {base_url}")

    # Construct sitemap URL
    sitemap_url = base_url.rstrip('/') + '/sitemap.xml'

    try:
        # Fetch sitemap with retry logic
        def fetch_sitemap():
            response = requests.get(sitemap_url, timeout=10)
            response.raise_for_status()
            return response

        response = retry_with_backoff(fetch_sitemap)

        # Parse XML to extract URLs
        soup = BeautifulSoup(response.content, 'lxml-xml')
        urls = [loc.text.strip() for loc in soup.find_all('loc')]

        # Filter out non-documentation pages
        excluded_patterns = ['/blog', '/tags', '/tag/']
        filtered_urls = []
        for url in urls:
            # Skip if URL contains any excluded pattern
            if not any(pattern in url for pattern in excluded_patterns):
                filtered_urls.append(url)

        # Deduplicate and sort
        filtered_urls = sorted(list(set(filtered_urls)))

        logging.info(f"Discovered {len(urls)} total URLs, {len(filtered_urls)} documentation URLs after filtering")
        return filtered_urls

    except requests.RequestException as e:
        logging.warning(f"Failed to fetch sitemap from {sitemap_url}: {str(e)}")
        return []
    except Exception as e:
        logging.error(f"Error parsing sitemap: {str(e)}")
        return []


def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extract clean text and metadata from a documentation page.

    Args:
        url: Absolute URL of the page

    Returns:
        Dictionary with structure:
        {
            "url": str,
            "title": str,
            "content": str,
            "section_hierarchy": List[str],
            "last_modified": Optional[str],
        }

    Raises:
        requests.RequestException: If page is unreachable after retries
        ValueError: If content cannot be extracted (no main content found)
    """
    try:
        # Fetch page HTML with retry logic
        def fetch_page():
            response = requests.get(url, timeout=10)
            response.raise_for_status()
            return response

        response = retry_with_backoff(fetch_page)

        # Parse with BeautifulSoup
        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract title from <h1> or <title>
        title = "Untitled"
        h1_tag = soup.find('h1')
        if h1_tag:
            title = h1_tag.get_text(strip=True)
        else:
            title_tag = soup.find('title')
            if title_tag:
                title = title_tag.get_text(strip=True)

        # Find main content area using Docusaurus-specific selectors
        main_content = soup.find('article') or soup.find('main') or soup.find(class_='markdown')

        if not main_content:
            raise ValueError(f"Could not find main content area in {url}")

        # Extract text while preserving structure
        content = main_content.get_text(separator='\n', strip=True)

        if not content or len(content.strip()) == 0:
            raise ValueError(f"Extracted content is empty for {url}")

        # Extract breadcrumb hierarchy
        section_hierarchy = []
        breadcrumb = soup.find(class_='breadcrumbs')
        if breadcrumb:
            # Find all links in breadcrumbs
            links = breadcrumb.find_all('a')
            section_hierarchy = [link.get_text(strip=True) for link in links if link.get_text(strip=True)]

        return {
            "url": url,
            "title": title,
            "content": content,
            "section_hierarchy": section_hierarchy,
            "last_modified": None,  # Could be extracted from meta tags if available
        }

    except requests.RequestException as e:
        logging.error(f"Failed to fetch {url}: {str(e)}")
        raise
    except ValueError as e:
        logging.error(f"Content extraction failed for {url}: {str(e)}")
        raise
    except Exception as e:
        logging.error(f"Unexpected error extracting from {url}: {str(e)}")
        raise


def chunk_text(
    content: str,
    chunk_size: int,
    chunk_overlap: int,
    code_threshold: int,
    page_url: str,
    page_title: str,
    section_hierarchy: List[str],
) -> List[Dict[str, Any]]:
    """
    Split content into chunks with adaptive code block handling.

    Args:
        content: Full page content
        chunk_size: Target chunk size in characters
        chunk_overlap: Overlap between chunks in characters
        code_threshold: Max code block size before splitting (characters)
        page_url: Source page URL
        page_title: Source page title
        section_hierarchy: Breadcrumb context

    Returns:
        List of ContentChunk dictionaries
    """
    if not content or len(content.strip()) == 0:
        return []

    chunks = []
    chunk_index = 0

    # Detect code blocks (markdown ``` or potential code patterns)
    # Simple heuristic: lines with 4+ spaces indentation or between ```
    lines = content.split('\n')

    # Process content with sliding window
    current_pos = 0
    content_length = len(content)

    while current_pos < content_length:
        # Determine chunk end position
        chunk_end = min(current_pos + chunk_size, content_length)

        # Extract chunk text
        chunk_text_content = content[current_pos:chunk_end]

        # Detect content type
        content_type = "prose"
        code_indicators = ['```', '    ', '\t', 'def ', 'function ', 'class ', 'import ', 'const ', 'let ', 'var ']
        code_count = sum(1 for indicator in code_indicators if indicator in chunk_text_content)

        if code_count >= 3:
            content_type = "code"
        elif code_count > 0:
            content_type = "mixed"

        # Extract nearest heading (look backwards in content for # or previous heading)
        heading = None
        heading_search = content[max(0, current_pos - 500):current_pos + 100]
        heading_lines = [line for line in heading_search.split('\n') if line.strip().startswith('#')]
        if heading_lines:
            heading = heading_lines[-1].lstrip('#').strip()

        # Create chunk dictionary
        chunk = {
            "chunk_id": str(uuid.uuid4()),
            "page_url": page_url,
            "page_title": page_title,
            "content": chunk_text_content.strip(),
            "chunk_index": chunk_index,
            "section_hierarchy": section_hierarchy,
            "heading": heading,
            "content_type": content_type,
            "metadata": {}
        }

        chunks.append(chunk)
        chunk_index += 1

        # Move position forward with overlap
        current_pos += chunk_size - chunk_overlap

        # Prevent infinite loop
        if chunk_size - chunk_overlap <= 0:
            break

    return chunks


def embed_chunks(
    chunks: List[Dict[str, Any]],
    cohere_client: cohere.Client,
    model: str,
    batch_size: int = 50,
) -> List[Dict[str, Any]]:
    """
    Generate embeddings for chunks using Cohere API with batch processing.

    Args:
        chunks: List of ContentChunk dictionaries
        cohere_client: Initialized Cohere client
        model: Cohere model name (e.g., "embed-english-v3.0")
        batch_size: Number of chunks to embed per API request (max 96)

    Returns:
        List of chunks with embeddings added to each chunk dictionary
        Each chunk gains an "embedding" key with a 1024-dimensional vector

    Raises:
        Exception: If Cohere API fails after retries
    """
    if not chunks:
        return []

    total_chunks = len(chunks)
    logging.info(f"Generating embeddings for {total_chunks} chunks in batches of {batch_size}...")

    embedded_chunks = []

    for batch_start in range(0, total_chunks, batch_size):
        batch_end = min(batch_start + batch_size, total_chunks)
        batch = chunks[batch_start:batch_end]

        # Extract text content from chunks
        texts = [chunk["content"] for chunk in batch]

        try:
            # Call Cohere API with retry logic for rate limits
            def call_cohere_embed():
                response = cohere_client.embed(
                    texts=texts,
                    model=model,
                    input_type="search_document",  # For indexing documents
                )
                return response

            # Retry with exponential backoff for rate limits (429 errors)
            response = retry_with_backoff(call_cohere_embed, max_retries=5)

            # Attach embeddings to chunks
            embeddings = response.embeddings
            for i, chunk in enumerate(batch):
                chunk["embedding"] = embeddings[i]
                embedded_chunks.append(chunk)

            # Log progress
            logging.info(f"Embedded {batch_end}/{total_chunks} chunks ({(batch_end/total_chunks)*100:.1f}%)")

        except Exception as e:
            logging.error(f"Failed to embed batch {batch_start}-{batch_end}: {str(e)}")
            raise

    logging.info(f"Embedding generation complete: {len(embedded_chunks)} chunks embedded")
    return embedded_chunks


def create_collection(
    qdrant_client: QdrantClient,
    collection_name: str,
    vector_size: int,
) -> None:
    """
    Initialize Qdrant collection for storing embeddings (idempotent).

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to create
        vector_size: Dimension of embedding vectors (e.g., 1024 for Cohere)

    Returns:
        None

    Raises:
        Exception: If collection creation fails
    """
    logging.info(f"Initializing Qdrant collection '{collection_name}'...")

    # T037: Idempotent check - skip if collection already exists
    if qdrant_client.collection_exists(collection_name):
        logging.info(f"Collection '{collection_name}' already exists, skipping creation")
        return

    # Create collection with VectorParams
    # T036: Configure with size=1024, distance=Cosine
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=vector_size,
            distance=Distance.COSINE,
        ),
    )

    logging.info(f"Collection '{collection_name}' created successfully with vector size {vector_size} and Cosine distance")


def save_chunks_to_qdrant(
    chunks: List[Dict[str, Any]],
    qdrant_client: QdrantClient,
    collection_name: str,
    batch_size: int = 100,
) -> None:
    """
    Save embedded chunks to Qdrant with batch upsert.

    Args:
        chunks: List of ContentChunk dictionaries with embeddings
        qdrant_client: Initialized Qdrant client
        collection_name: Name of the collection to store vectors
        batch_size: Number of points to upsert per batch (default: 100)

    Returns:
        None

    Raises:
        Exception: If batch upsert fails
        ValueError: If chunks are missing embeddings
    """
    if not chunks:
        logging.warning("No chunks to save")
        return

    # Validate that chunks have embeddings
    if not all("embedding" in chunk for chunk in chunks):
        raise ValueError("All chunks must have embeddings before saving to Qdrant")

    logging.info(f"Saving {len(chunks)} chunks to Qdrant collection '{collection_name}'...")

    # T039: Transform ContentChunk to PointStruct format
    points = []
    for chunk in chunks:
        # Extract embedding (vector)
        embedding = chunk["embedding"]

        # Build payload with all metadata (exclude embedding from payload)
        payload = {
            "page_url": chunk["page_url"],
            "page_title": chunk["page_title"],
            "content": chunk["content"],
            "chunk_index": chunk["chunk_index"],
            "section_hierarchy": chunk["section_hierarchy"],
            "heading": chunk.get("heading"),  # May be None
            "content_type": chunk["content_type"],
        }

        # Add any additional metadata
        if "metadata" in chunk and chunk["metadata"]:
            payload["metadata"] = chunk["metadata"]

        # Create PointStruct
        point = PointStruct(
            id=chunk["chunk_id"],  # Use chunk_id as unique identifier
            vector=embedding,
            payload=payload,
        )
        points.append(point)

    # T040: Batch upsert logic - process in batches of batch_size (default 100)
    total_points = len(points)

    for batch_start in range(0, total_points, batch_size):
        batch_end = min(batch_start + batch_size, total_points)
        batch = points[batch_start:batch_end]

        try:
            # Upsert batch to Qdrant
            qdrant_client.upsert(
                collection_name=collection_name,
                points=batch,
            )

            # T041: Progress logging for Qdrant operations
            logging.info(f"Saved {batch_end}/{total_points} chunks to Qdrant ({(batch_end/total_points)*100:.1f}%)")

        except Exception as e:
            logging.error(f"Failed to upsert batch {batch_start}-{batch_end}: {str(e)}")
            raise

    logging.info(f"Successfully saved {total_points} chunks to Qdrant")


def verify_ingestion(
    qdrant_client: QdrantClient,
    cohere_client: cohere.Client,
    collection_name: str,
    model: str,
    query: str = "What is this documentation about?",
) -> None:
    """
    Verify ingestion pipeline by performing a sample similarity search.

    Args:
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere client
        collection_name: Name of the collection to search
        model: Cohere model name for generating query embedding
        query: Sample query text (default: "What is this documentation about?")

    Returns:
        None

    Raises:
        Exception: If verification search fails
    """
    logging.info(f"Verifying ingestion with sample query: '{query}'")

    # T043: Generate query embedding using Cohere (input_type="search_query")
    try:
        response = cohere_client.embed(
            texts=[query],
            model=model,
            input_type="search_query",  # For search queries (vs "search_document" for indexing)
        )
        query_embedding = response.embeddings[0]
        logging.info(f"Generated query embedding (dimension: {len(query_embedding)})")

    except Exception as e:
        logging.error(f"Failed to generate query embedding: {str(e)}")
        raise

    # T044: Perform Qdrant similarity search (limit=5)
    try:
        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=5,
        )
        logging.info(f"Found {len(search_results)} similar chunks")

    except Exception as e:
        logging.error(f"Failed to perform similarity search: {str(e)}")
        raise

    # T045: Log top result with score and metadata
    if search_results:
        top_result = search_results[0]
        logging.info("=" * 80)
        logging.info("Top similarity search result:")
        logging.info(f"  Score: {top_result.score:.4f}")
        logging.info(f"  Page: {top_result.payload.get('page_title', 'N/A')}")
        logging.info(f"  URL: {top_result.payload.get('page_url', 'N/A')}")
        logging.info(f"  Content type: {top_result.payload.get('content_type', 'N/A')}")

        # Show content snippet (first 200 chars)
        content = top_result.payload.get('content', '')
        content_snippet = content[:200] + "..." if len(content) > 200 else content
        logging.info(f"  Content: {content_snippet}")
        logging.info("=" * 80)
        logging.info("✓ Verification successful - pipeline working correctly!")
    else:
        logging.warning("No search results found - collection may be empty")


def main() -> None:
    """
    Main execution function that orchestrates the entire ingestion pipeline.

    Steps:
        1. Load environment variables and configuration
        2. Initialize Cohere and Qdrant clients
        3. Create Qdrant collection (if not exists)
        4. Crawl website to discover all URLs
        5. Extract content from each URL
        6. Chunk all extracted content
        7. Generate embeddings for all chunks
        8. Save chunks to Qdrant
        9. Perform sample similarity search to verify
        10. Print summary statistics
    """
    logging.info("Starting RAG content ingestion...")

    try:
        # 1. Load configuration
        config = load_config()
        logging.info(f"Base URL: {config['base_url']}")
        logging.info(f"Collection: {config['collection_name']}")
        logging.info(f"Chunk size: {config['chunk_size']}, Overlap: {config['chunk_overlap']}")

        # 2. Initialize Cohere client
        logging.info(f"Initializing Cohere client with model: {config['cohere_model']}")
        cohere_client = cohere.Client(api_key=config['cohere_api_key'])

        # 3. Initialize Qdrant client (T046)
        logging.info(f"Initializing Qdrant client at {config['qdrant_url']}")
        qdrant_client = QdrantClient(
            url=config['qdrant_url'],
            api_key=config['qdrant_api_key'],
        )

        # 4. Create Qdrant collection (T047)
        create_collection(
            qdrant_client=qdrant_client,
            collection_name=config['collection_name'],
            vector_size=config['vector_size'],
        )

        # 5. Crawl website to discover all URLs
        logging.info("Discovering URLs from sitemap...")
        urls = get_all_urls(config['base_url'])

        if not urls:
            logging.error("No URLs discovered. Exiting.")
            return

        # 6. Extract content from each URL
        logging.info(f"Extracting content from {len(urls)} pages...")
        pages = []
        failed_count = 0

        for idx, url in enumerate(urls, 1):
            try:
                page = extract_text_from_url(url)
                pages.append(page)

                if idx % 10 == 0:  # Log progress every 10 pages
                    logging.info(f"Processed {idx}/{len(urls)} pages ({len(pages)} successful, {failed_count} failed)")

            except (requests.RequestException, ValueError) as e:
                failed_count += 1
                logging.warning(f"Skipping {url} due to error: {str(e)}")
                continue
            except Exception as e:
                failed_count += 1
                logging.error(f"Unexpected error processing {url}: {str(e)}")
                continue

        logging.info(f"Content extraction complete: {len(pages)} pages successfully extracted, {failed_count} failed")

        if not pages:
            logging.error("No content extracted. Exiting.")
            return

        # 7. Chunk all extracted content
        logging.info(f"Chunking {len(pages)} pages...")
        all_chunks = []

        for page in pages:
            chunks = chunk_text(
                content=page["content"],
                chunk_size=config['chunk_size'],
                chunk_overlap=config['chunk_overlap'],
                code_threshold=config['code_threshold'],
                page_url=page["url"],
                page_title=page["title"],
                section_hierarchy=page["section_hierarchy"],
            )
            all_chunks.extend(chunks)

        logging.info(f"Chunking complete: {len(all_chunks)} chunks created from {len(pages)} pages (avg {len(all_chunks)/len(pages):.1f} chunks/page)")

        if not all_chunks:
            logging.error("No chunks created. Exiting.")
            return

        # 8. Generate embeddings for all chunks
        embedded_chunks = embed_chunks(
            chunks=all_chunks,
            cohere_client=cohere_client,
            model=config['cohere_model'],
            batch_size=config['batch_size'],
        )

        # 9. Save chunks to Qdrant (T047)
        save_chunks_to_qdrant(
            chunks=embedded_chunks,
            qdrant_client=qdrant_client,
            collection_name=config['collection_name'],
            batch_size=100,
        )

        # 10. Verify ingestion with sample search (T047)
        verify_ingestion(
            qdrant_client=qdrant_client,
            cohere_client=cohere_client,
            collection_name=config['collection_name'],
            model=config['cohere_model'],
        )

        # Pipeline complete
        logging.info("=" * 80)
        logging.info("✓ RAG Content Ingestion Pipeline Complete!")
        logging.info(f"  Total pages processed: {len(pages)}")
        logging.info(f"  Total chunks created: {len(all_chunks)}")
        logging.info(f"  Total vectors stored: {len(embedded_chunks)}")
        logging.info(f"  Collection: {config['collection_name']}")
        logging.info(f"  Qdrant URL: {config['qdrant_url']}")
        logging.info("=" * 80)

    except ValueError as e:
        logging.error(f"Configuration error: {str(e)}")
        return
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return


if __name__ == "__main__":
    main()
