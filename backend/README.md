# RAG Content Ingestion Pipeline

Automated pipeline for extracting documentation from Docusaurus websites, generating embeddings using Cohere, and storing them in Qdrant Cloud for semantic search.

## Features

- **Automated Content Extraction**: Crawls Docusaurus sitemaps and extracts clean text content
- **Adaptive Chunking**: Intelligent chunking with code block detection and overlap
- **Cohere Embeddings**: High-quality 1024-dimensional embeddings using `embed-english-v3.0`
- **Qdrant Vector Storage**: Scalable cloud-based vector database with cosine similarity
- **Batch Processing**: Efficient API usage with configurable batch sizes
- **Retry Logic**: Exponential backoff for network failures and rate limits
- **Progress Tracking**: Detailed logging throughout the pipeline
- **End-to-End Verification**: Sample similarity search to validate the pipeline

## Prerequisites

- Python 3.11+
- [uv](https://github.com/astral-sh/uv) package manager
- Cohere API key ([sign up](https://cohere.com/))
- Qdrant Cloud account ([sign up](https://cloud.qdrant.io/))

## Installation

1. **Initialize project with uv**:
   ```bash
   cd backend
   uv python pin 3.11
   uv sync
   ```

2. **Create `.env` file** (copy from `.env.example`):
   ```bash
   cp .env.example .env
   ```

3. **Configure environment variables** (see Configuration section below)

## Configuration

Create a `.env` file in the `backend/` directory with the following variables:

### Required Variables

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# Target Documentation Site
BASE_URL=https://docs.example.com
```

### Optional Variables (with defaults)

```env
# Chunking Configuration
CHUNK_SIZE=1000              # Characters per chunk (1-2000)
CHUNK_OVERLAP=200            # Overlap between chunks (0-CHUNK_SIZE)
CODE_THRESHOLD=500           # Code block split threshold (1-CHUNK_SIZE)

# Model Configuration
COHERE_MODEL=embed-english-v3.0    # Cohere embedding model
COLLECTION_NAME=rag_embedding      # Qdrant collection name
BATCH_SIZE=50                      # Embeddings per API request (1-96)
```

## Usage

### Run the complete pipeline:

```bash
uv run python main.py
```

### Expected Output:

```
INFO: Configuration loaded successfully
INFO: Initializing Cohere client with model: embed-english-v3.0
INFO: Initializing Qdrant client at https://...
INFO: Initializing Qdrant collection 'rag_embedding'...
INFO: Collection 'rag_embedding' created successfully with vector size 1024 and Cosine distance
INFO: Discovering URLs from sitemap...
INFO: Discovered 42 total URLs, 38 documentation URLs after filtering
INFO: Extracting content from 38 pages...
INFO: Processed 10/38 pages (10 successful, 0 failed)
...
INFO: Content extraction complete: 38 pages successfully extracted, 0 failed
INFO: Chunking 38 pages...
INFO: Chunking complete: 156 chunks created from 38 pages (avg 4.1 chunks/page)
INFO: Generating embeddings for 156 chunks in batches of 50...
INFO: Embedded 50/156 chunks (32.1%)
...
INFO: Embedding generation complete: 156 chunks embedded
INFO: Saving 156 chunks to Qdrant collection 'rag_embedding'...
INFO: Saved 100/156 chunks to Qdrant (64.1%)
INFO: Saved 156/156 chunks to Qdrant (100.0%)
INFO: Successfully saved 156 chunks to Qdrant
INFO: Verifying ingestion with sample query: 'What is this documentation about?'
INFO: Generated query embedding (dimension: 1024)
INFO: Found 5 similar chunks
INFO: ================================================================================
INFO: Top similarity search result:
INFO:   Score: 0.8542
INFO:   Page: Introduction
INFO:   URL: https://docs.example.com/intro
INFO:   Content type: prose
INFO:   Content: This documentation covers the fundamental concepts of...
INFO: ================================================================================
INFO: ✓ Verification successful - pipeline working correctly!
INFO: ================================================================================
INFO: ✓ RAG Content Ingestion Pipeline Complete!
INFO:   Total pages processed: 38
INFO:   Total chunks created: 156
INFO:   Total vectors stored: 156
INFO:   Collection: rag_embedding
INFO:   Qdrant URL: https://...
INFO: ================================================================================
```

## Project Structure

```
backend/
├── main.py              # Complete pipeline implementation
├── .env                 # Environment configuration (gitignored)
├── .env.example         # Environment template
├── pyproject.toml       # uv project configuration
├── uv.lock             # Dependency lock file
└── README.md           # This file
```

## Architecture

The pipeline follows a 10-step process:

1. **Load Configuration**: Validate environment variables
2. **Initialize Clients**: Set up Cohere and Qdrant clients
3. **Create Collection**: Initialize Qdrant collection (idempotent)
4. **Crawl Sitemap**: Discover all documentation URLs
5. **Extract Content**: Parse HTML and extract clean text
6. **Chunk Text**: Split content with adaptive chunking
7. **Generate Embeddings**: Create 1024-dim vectors with Cohere
8. **Store Vectors**: Batch upsert to Qdrant with metadata
9. **Verify Ingestion**: Sample similarity search
10. **Summary**: Report statistics

## Functions

### Core Functions

- `load_config()`: Load and validate environment variables
- `retry_with_backoff()`: Exponential backoff for network calls
- `get_all_urls()`: Crawl sitemap and filter documentation URLs
- `extract_text_from_url()`: Parse HTML and extract content + metadata
- `chunk_text()`: Adaptive chunking with code detection
- `embed_chunks()`: Generate embeddings with Cohere API
- `create_collection()`: Initialize Qdrant collection (idempotent)
- `save_chunks_to_qdrant()`: Batch upsert vectors with metadata
- `verify_ingestion()`: Sample similarity search for validation
- `main()`: Orchestrate the complete pipeline

## Troubleshooting

### Common Issues

**Missing Environment Variables**:
```
ValueError: Missing required environment variables: COHERE_API_KEY, QDRANT_URL
```
→ Ensure all required variables are set in `.env`

**Invalid Configuration**:
```
ValueError: chunk_size must be between 1 and 2000, got 5000
```
→ Check configuration ranges in the Configuration section

**Network Errors**:
```
WARNING: Attempt 1/3 failed: Connection timeout. Retrying in 1s...
```
→ The pipeline automatically retries with exponential backoff (1s, 2s, 4s)

**Cohere Rate Limits**:
```
ERROR: Failed to embed batch 0-50: Rate limit exceeded
```
→ Reduce `BATCH_SIZE` in `.env` or wait and retry

**Qdrant Connection Issues**:
```
ERROR: Failed to upsert batch 0-100: Connection refused
```
→ Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct

**Empty Sitemap**:
```
WARNING: No URLs discovered. Exiting.
```
→ Verify `BASE_URL` points to a valid Docusaurus site with `/sitemap.xml`

**Content Extraction Failures**:
```
WARNING: Skipping https://... due to error: Could not find main content area
```
→ The page may not be a standard Docusaurus page; non-critical if only a few pages fail

## Edge Cases and Limitations

### Known Limitations

1. **Static HTML Only**: The pipeline uses `requests` + `BeautifulSoup`, which cannot handle JavaScript-rendered content
   - **Workaround**: Use sites with server-side rendered content (e.g., standard Docusaurus)
   - **Alternative**: Consider using Playwright/Selenium for JS-heavy sites

2. **Docusaurus-Specific**: Optimized for Docusaurus site structure
   - Uses `/sitemap.xml` for discovery
   - Expects `<article>`, `<main>`, or `.markdown` containers
   - **Workaround**: Modify `extract_text_from_url()` for other frameworks

3. **Rate Limits**:
   - **Cohere**: Max 96 texts per embed request, rate limits vary by plan
   - **Qdrant Cloud**: Rate limits depend on cluster tier
   - **Mitigation**: Built-in exponential backoff handles transient limits

4. **Large Sites**: Sites with 1000+ pages may take significant time
   - **Estimate**: ~2-3 seconds per page (extraction + embedding + storage)
   - **Example**: 1000 pages ≈ 30-50 minutes
   - **Workaround**: Filter URLs more aggressively or process in batches

5. **Code Block Handling**: Code blocks are detected heuristically
   - Uses indicators like ` ``` `, indentation, keywords
   - May not perfectly detect all code blocks
   - **Impact**: Minor; affects `content_type` metadata only

6. **No Incremental Updates**: Full re-ingestion on each run
   - **Limitation**: No change detection or delta updates
   - **Impact**: Re-processing unchanged pages wastes API calls
   - **Future Enhancement**: Add last-modified checking

7. **No Authentication**: Cannot access password-protected documentation
   - **Workaround**: Use publicly accessible sites or run locally

### Best Practices

- **Test with Small Site First**: Validate with a small Docusaurus site (e.g., https://docusaurus.io)
- **Monitor Logs**: Watch for extraction failures and adjust selectors if needed
- **Cost Management**:
  - Cohere: ~$0.10 per 1M tokens (varies by plan)
  - Qdrant: Free tier available, paid tiers based on storage/requests
- **Idempotent Collection**: Safe to re-run; collection creation is idempotent
- **Batch Size Tuning**:
  - Larger batches (50-96) = fewer API calls but higher rate limit risk
  - Smaller batches (10-25) = more robust but slower

## Dependencies

Main dependencies (see `pyproject.toml` for full list):

- `cohere>=5.20.1`: Cohere API client for embeddings
- `qdrant-client>=1.16.2`: Qdrant vector database client
- `requests>=2.32.5`: HTTP library for sitemap/page fetching
- `beautifulsoup4>=4.14.3`: HTML parsing
- `lxml>=6.0.2`: Fast XML/HTML parser
- `python-dotenv>=1.2.1`: Environment variable management

## Testing

### Manual Testing Checklist

- [ ] Pipeline runs without errors on a small Docusaurus site
- [ ] All pages are discovered from sitemap
- [ ] Content is extracted successfully (check logs for failures)
- [ ] Embeddings are generated without rate limit errors
- [ ] Vectors are stored in Qdrant (verify in Qdrant dashboard)
- [ ] Verification search returns relevant results (score > 0.5)
- [ ] Re-running pipeline doesn't duplicate vectors (collection idempotent)

### Test Sites

- **Small**: https://docusaurus.io (~50 pages)
- **Medium**: https://docs.cohere.com (~200 pages)
- **Large**: Custom documentation site (500+ pages)

## Next Steps

After successful ingestion, you can:

1. **Query the Vector Database**: Use Qdrant's search API or Python client
2. **Build RAG Application**: Combine with LLM for question-answering
3. **Set Up Refresh Schedule**: Cron job or CI/CD for periodic re-ingestion
4. **Add Filtering**: Use Qdrant's payload filters for scoped searches
5. **Monitor Performance**: Track search latency and relevance metrics

## License

This project is part of the AI Humanoid Robotics Book RAG system.

## Support

For issues or questions:
- Check the Troubleshooting section above
- Review error messages in logs (stderr)
- Verify `.env` configuration matches requirements
- Test with a known-working Docusaurus site first
