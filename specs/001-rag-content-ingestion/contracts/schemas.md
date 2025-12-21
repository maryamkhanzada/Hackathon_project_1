# API Contracts and Schemas

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-21
**Purpose**: Define function signatures, types, and contracts for `backend/main.py`

## Module Organization

All functions are implemented in a single file (`backend/main.py`) with logical grouping:

1. **Configuration**: Environment loading and validation
2. **Web Crawling**: URL discovery from sitemap
3. **Content Extraction**: HTML parsing and text extraction
4. **Chunking**: Adaptive text splitting with code awareness
5. **Embedding**: Cohere API integration
6. **Qdrant Operations**: Collection management and vector storage
7. **Main Orchestration**: Pipeline execution and verification

---

## 1. Configuration

### load_config()

```python
def load_config() -> Dict[str, Any]:
    """
    Load configuration from environment variables with defaults.

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

    Example:
        >>> config = load_config()
        >>> print(config["base_url"])
        "https://docs.example.com"
    """
    pass
```

**Contract**:
- Must validate all required env vars are present
- Must apply defaults for optional parameters
- Must validate parameter ranges (chunk_size > 0, etc.)
- Returns validated config dictionary

---

## 2. Web Crawling

### get_all_urls()

```python
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

    Implementation:
        1. Construct sitemap URL: {base_url}/sitemap.xml
        2. Fetch sitemap with retry logic (max 3 attempts)
        3. Parse XML to extract all <loc> tags
        4. Filter for documentation URLs (exclude /blog, /tags if present)
        5. Return unique, sorted URLs

    Example:
        >>> urls = get_all_urls("https://docs.example.com")
        >>> print(len(urls))
        142
        >>> print(urls[0])
        "https://docs.example.com/docs/intro"
    """
    pass
```

**Contract**:
- Returns empty list (not error) if sitemap not found
- Deduplicates URLs before returning
- Filters out non-documentation pages (blog, tags)
- Retries network requests up to 3 times with exponential backoff

---

## 3. Content Extraction

### extract_text_from_url()

```python
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

    Implementation:
        1. Fetch page HTML with retry logic
        2. Parse with BeautifulSoup
        3. Extract title from <h1> or <title>
        4. Find main content area (<article>, <main>, .markdown)
        5. Extract text preserving structure (newlines, code blocks)
        6. Extract breadcrumbs from .breadcrumbs class
        7. Return structured dictionary

    Example:
        >>> page = extract_text_from_url("https://docs.example.com/api/auth")
        >>> print(page["title"])
        "Authentication"
        >>> print(page["section_hierarchy"])
        ["API Reference", "Authentication"]
        >>> print(len(page["content"]))
        3542
    """
    pass
```

**Contract**:
- Raises ValueError if no main content area found
- Preserves code block formatting (backticks, indentation)
- Returns empty list for section_hierarchy if no breadcrumbs
- Content must be non-empty string
- Retries failed requests up to 3 times

---

## 4. Chunking

### chunk_text()

```python
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
        List of ContentChunk dictionaries (see data-model.md)

    Implementation:
        1. Detect code blocks (markdown ``` or HTML <pre>)
        2. For each code block:
           - If size < code_threshold: keep with surrounding prose
           - If size >= code_threshold: extract as separate chunk
        3. For remaining prose: sliding window with overlap
        4. Assign chunk_id (UUID), chunk_index, content_type
        5. Detect nearest heading for each chunk
        6. Return list of ContentChunks

    Edge Cases:
        - Empty content: Return empty list
        - Content shorter than chunk_size: Return single chunk
        - Multiple consecutive code blocks: Merge if total < threshold
        - Code block at chunk boundary: Avoid splitting mid-block

    Example:
        >>> chunks = chunk_text(
        ...     content="# Intro\n\nSome text...\n\n```python\ncode\n```\n\nMore text...",
        ...     chunk_size=1000,
        ...     chunk_overlap=200,
        ...     code_threshold=500,
        ...     page_url="https://docs.example.com/guide",
        ...     page_title="Guide",
        ...     section_hierarchy=["Guides"]
        ... )
        >>> print(len(chunks))
        3
        >>> print(chunks[0]["content_type"])
        "mixed"
    """
    pass
```

**Contract**:
- Each chunk must have unique chunk_id (UUID)
- chunk_index starts at 0 and increments sequentially
- content_type must be "prose", "code", or "mixed"
- No chunk exceeds chunk_size + chunk_overlap (buffer)
- Adjacent chunks overlap by ~chunk_overlap characters
- Preserves code block integrity (no mid-block splits)

---

## 5. Embedding Generation

### embed_chunks()

```python
def embed_chunks(
    chunks: List[Dict[str, Any]],
    cohere_client,
    model: str,
    batch_size: int = 50,
) -> List[Dict[str, Any]]:
    """
    Generate embeddings for chunks using Cohere API in batches.

    Args:
        chunks: List of ContentChunk dictionaries
        cohere_client: Initialized Cohere client (cohere.Client)
        model: Cohere model name (e.g., "embed-english-v3.0")
        batch_size: Number of chunks per API request (max 96)

    Returns:
        List of chunks with added "embedding" field (List[float])

    Raises:
        cohere.CohereAPIError: If API request fails after retries
        ValueError: If chunks list is empty

    Implementation:
        1. Validate chunks list is non-empty
        2. Extract text content from each chunk
        3. Process in batches of batch_size (max 96)
        4. Call cohere.embed() with input_type="search_document"
        5. Handle rate limits with exponential backoff
        6. Attach embeddings to corresponding chunks
        7. Return updated chunks

    Rate Limiting:
        - Retry on 429 (Too Many Requests) with exponential backoff
        - Max retries: 3
        - Backoff: 1s, 2s, 4s

    Example:
        >>> import cohere
        >>> co = cohere.Client(api_key="...")
        >>> chunks_with_embeddings = embed_chunks(
        ...     chunks=content_chunks,
        ...     cohere_client=co,
        ...     model="embed-english-v3.0",
        ...     batch_size=50
        ... )
        >>> print(len(chunks_with_embeddings[0]["embedding"]))
        1024
    """
    pass
```

**Contract**:
- Processes chunks in batches to respect rate limits
- Each chunk gets "embedding" field with 1024 floats
- Retries failed requests with exponential backoff
- Logs progress (e.g., "Embedded 50/150 chunks")
- Raises error if any batch fails after retries

---

## 6. Qdrant Operations

### create_collection()

```python
def create_collection(
    qdrant_client,
    collection_name: str,
    vector_size: int,
    distance: str = "Cosine",
) -> None:
    """
    Create Qdrant collection if it doesn't exist.

    Args:
        qdrant_client: Initialized Qdrant client (qdrant_client.QdrantClient)
        collection_name: Name for the collection
        vector_size: Dimension of vectors (e.g., 1024)
        distance: Distance metric ("Cosine", "Dot", "Euclidean")

    Raises:
        Exception: If collection creation fails

    Implementation:
        1. Check if collection exists
        2. If exists: Log message and return (idempotent)
        3. If not exists: Create with VectorParams
        4. Log success message

    Example:
        >>> from qdrant_client import QdrantClient
        >>> client = QdrantClient(url="...", api_key="...")
        >>> create_collection(client, "rag_embedding", 1024, "Cosine")
        INFO: Collection 'rag_embedding' created successfully
    """
    pass
```

**Contract**:
- Idempotent (safe to call multiple times)
- Uses Distance enum from qdrant_client.models
- Logs informative messages
- Validates collection_name is alphanumeric + underscores

---

### save_chunks_to_qdrant()

```python
def save_chunks_to_qdrant(
    qdrant_client,
    collection_name: str,
    chunks_with_embeddings: List[Dict[str, Any]],
    batch_size: int = 100,
) -> None:
    """
    Save embedded chunks to Qdrant collection in batches.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Target collection name
        chunks_with_embeddings: Chunks with "embedding" field
        batch_size: Number of points per upsert request

    Raises:
        Exception: If upsert operation fails

    Implementation:
        1. Validate chunks have "embedding" field
        2. Transform chunks to Qdrant PointStruct format:
           - id: chunk_id
           - vector: embedding
           - payload: {page_url, page_title, content, chunk_index, ...}
        3. Upsert in batches of batch_size
        4. Log progress (e.g., "Saved 100/500 chunks")
        5. Verify total count matches input

    Example:
        >>> save_chunks_to_qdrant(
        ...     qdrant_client=client,
        ...     collection_name="rag_embedding",
        ...     chunks_with_embeddings=chunks,
        ...     batch_size=100
        ... )
        INFO: Saved 100/500 chunks to rag_embedding
        INFO: Saved 200/500 chunks to rag_embedding
        ...
        INFO: Successfully saved 500 chunks
    """
    pass
```

**Contract**:
- Transforms ContentChunk to Qdrant PointStruct
- Batches upserts for performance
- Logs progress every batch
- Verifies final count matches input
- Uses upsert (not insert) to allow re-ingestion

---

### verify_ingestion()

```python
def verify_ingestion(
    qdrant_client,
    cohere_client,
    collection_name: str,
    model: str,
    query: str = "How do I get started?",
) -> List[Dict[str, Any]]:
    """
    Perform a sample similarity search to verify successful ingestion.

    Args:
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere client
        collection_name: Collection to search
        model: Cohere model for query embedding
        query: Sample search query

    Returns:
        List of search results with scores and metadata

    Implementation:
        1. Generate embedding for query using Cohere
        2. Perform similarity search in Qdrant (limit=5)
        3. Return results with scores and payloads
        4. Log top result for verification

    Example:
        >>> results = verify_ingestion(
        ...     qdrant_client=client,
        ...     cohere_client=co,
        ...     collection_name="rag_embedding",
        ...     model="embed-english-v3.0",
        ...     query="How do I authenticate?"
        ... )
        >>> print(results[0]["score"])
        0.87
        >>> print(results[0]["payload"]["page_title"])
        "Authentication"
    """
    pass
```

**Contract**:
- Uses input_type="search_query" for query embedding
- Returns top 5 results by default
- Logs top result for manual verification
- Returns empty list if no results found (not an error)

---

## 7. Main Orchestration

### main()

```python
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

    Error Handling:
        - Missing env vars: Exit with error message
        - Network failures: Retry with backoff, skip page if fails
        - API errors: Exit with error message
        - Empty results: Log warning but continue

    Logging:
        - INFO: Progress updates (URLs found, pages processed, chunks created)
        - WARNING: Skipped pages, retries
        - ERROR: Fatal errors (missing credentials, API failures)

    Example Output:
        INFO: Loaded configuration
        INFO: Connected to Cohere API
        INFO: Connected to Qdrant Cloud
        INFO: Collection 'rag_embedding' ready
        INFO: Discovered 142 URLs from sitemap
        INFO: Processed 142/142 pages
        INFO: Created 1834 chunks
        INFO: Embedded 1834/1834 chunks
        INFO: Saved 1834 chunks to Qdrant
        INFO: Verification query: "How do I get started?"
        INFO: Top result: "Getting Started" (score: 0.91)
        INFO: Ingestion complete!
    """
    pass
```

**Contract**:
- Exits with code 0 on success, 1 on fatal error
- Prints progress to stdout (INFO level)
- Handles errors gracefully (logs and continues where possible)
- Provides summary statistics at end
- Verifies ingestion with sample query

---

## Type Annotations

All functions should use Python type hints:

```python
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient

# Example function signature
def chunk_text(
    content: str,
    chunk_size: int,
    chunk_overlap: int,
    code_threshold: int,
    page_url: str,
    page_title: str,
    section_hierarchy: List[str],
) -> List[Dict[str, Any]]:
    ...
```

---

## Error Handling Patterns

### Network Retries

```python
import time
from requests.exceptions import RequestException

def retry_request(url: str, max_retries: int = 3) -> requests.Response:
    """Retry HTTP request with exponential backoff"""
    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()
            return response
        except RequestException as e:
            if attempt == max_retries - 1:
                raise
            wait_time = 2 ** attempt  # 1s, 2s, 4s
            logging.warning(f"Request failed (attempt {attempt+1}/{max_retries}), retrying in {wait_time}s...")
            time.sleep(wait_time)
```

### API Rate Limiting

```python
import cohere

def embed_with_retry(co: cohere.Client, texts: List[str], model: str) -> List[List[float]]:
    """Handle Cohere rate limits with backoff"""
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = co.embed(
                texts=texts,
                model=model,
                input_type="search_document"
            )
            return response.embeddings
        except cohere.CohereAPIError as e:
            if e.status_code == 429 and attempt < max_retries - 1:
                wait_time = 2 ** attempt
                logging.warning(f"Rate limited, retrying in {wait_time}s...")
                time.sleep(wait_time)
            else:
                raise
```

---

## Testing Contracts

Each function should be testable independently:

```python
# Test URL discovery
def test_get_all_urls():
    urls = get_all_urls("https://docusaurus.io")
    assert len(urls) > 0
    assert all(url.startswith("https://") for url in urls)

# Test content extraction
def test_extract_text_from_url():
    page = extract_text_from_url("https://docusaurus.io/docs")
    assert page["title"]
    assert page["content"]
    assert isinstance(page["section_hierarchy"], list)

# Test chunking
def test_chunk_text():
    chunks = chunk_text(
        content="# Test\n\nContent here...",
        chunk_size=100,
        chunk_overlap=20,
        code_threshold=50,
        page_url="https://test.com",
        page_title="Test",
        section_hierarchy=[]
    )
    assert len(chunks) > 0
    assert all("chunk_id" in c for c in chunks)
```

---

## Next Steps

1. Implement all functions in `backend/main.py` following these contracts
2. Add docstrings matching these specifications
3. Implement error handling as specified
4. Add logging statements for progress tracking
5. Test each function independently before integration
