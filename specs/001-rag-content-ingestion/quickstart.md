# Quickstart: RAG Content Ingestion

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-21
**Purpose**: Step-by-step guide to set up and run the RAG content ingestion pipeline

## Overview

This guide will help you:
1. Install required tools (`uv`)
2. Set up the Python environment
3. Configure API credentials
4. Run the ingestion pipeline
5. Verify successful ingestion

**Time to complete**: ~15 minutes

---

## Prerequisites

Before starting, ensure you have:

- **Python 3.11 or higher** installed
- **Git** installed (for cloning the repository)
- **Cohere API key** (free tier available at [https://cohere.com](https://cohere.com))
- **Qdrant Cloud account** (free tier at [https://qdrant.tech/cloud](https://qdrant.tech/cloud))
- **Target Docusaurus website URL** (publicly accessible)

---

## Step 1: Install UV

`uv` is a fast Python package manager that simplifies virtual environment and dependency management.

### macOS / Linux

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Windows (PowerShell)

```powershell
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### Verify Installation

```bash
uv --version
```

Expected output: `uv 0.x.x` or higher

---

## Step 2: Set Up the Project

### Navigate to Backend Directory

```bash
cd backend
```

If the `backend/` directory doesn't exist, create it:

```bash
mkdir backend
cd backend
```

### Initialize Python Project

```bash
# Initialize uv project
uv init

# Pin Python version to 3.11
uv python pin 3.11
```

This creates:
- `pyproject.toml` (project metadata)
- `.python-version` (Python version specification)

### Add Dependencies

```bash
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv lxml
```

This will:
- Install all dependencies
- Create `uv.lock` (lockfile for reproducibility)
- Set up `.venv/` virtual environment automatically

**Dependencies installed**:
- `cohere`: Cohere API client for embeddings
- `qdrant-client`: Qdrant Cloud integration
- `requests`: HTTP requests for web scraping
- `beautifulsoup4`: HTML parsing
- `python-dotenv`: Environment variable management
- `lxml`: Fast XML/HTML parser (used by BeautifulSoup)

---

## Step 3: Get API Credentials

### Cohere API Key

1. Go to [https://cohere.com](https://cohere.com)
2. Sign up or log in
3. Navigate to **Dashboard** → **API Keys**
4. Copy your API key (starts with `co_...` or similar)
5. **Free tier**: 100 API calls/minute, 1000 calls/month

### Qdrant Cloud Credentials

1. Go to [https://qdrant.tech/cloud](https://qdrant.tech/cloud)
2. Sign up for a free account
3. Create a new cluster:
   - Cluster name: `rag-cluster` (or any name)
   - Region: Choose closest to you
   - Tier: **Free** (1GB storage)
4. After cluster is created, go to **Cluster Details**:
   - Copy **Cluster URL** (e.g., `https://xyz-abc.qdrant.io`)
   - Create and copy **API Key** from the **API Keys** section

---

## Step 4: Configure Environment Variables

### Create `.env` File

In the `backend/` directory, create a `.env` file:

```bash
# backend/.env

# Required: Cohere API Key
COHERE_API_KEY=your_cohere_api_key_here

# Required: Qdrant Cloud URL and API Key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Required: Docusaurus website base URL
BASE_URL=https://docs.example.com

# Optional: Chunking parameters (defaults shown)
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
CODE_THRESHOLD=500

# Optional: Cohere model (default: embed-english-v3.0)
COHERE_MODEL=embed-english-v3.0

# Optional: Qdrant collection name (default: rag_embedding)
COLLECTION_NAME=rag_embedding

# Optional: Embedding batch size (default: 50, max: 96)
BATCH_SIZE=50
```

### Create `.env.example` Template

Create a template file for others (without secrets):

```bash
# backend/.env.example

COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
BASE_URL=https://docs.example.com
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
CODE_THRESHOLD=500
COHERE_MODEL=embed-english-v3.0
COLLECTION_NAME=rag_embedding
BATCH_SIZE=50
```

### Update `.gitignore`

Ensure `.env` is not committed to Git:

```bash
# Add to .gitignore (or create if not exists)
echo "backend/.env" >> ../.gitignore
echo "backend/.venv/" >> ../.gitignore
```

---

## Step 5: Implement the Ingestion Script

Create `backend/main.py` with the ingestion pipeline implementation.

**Note**: See `contracts/schemas.md` for detailed function specifications.

### Minimal Template

```python
# backend/main.py

import os
import logging
from typing import List, Dict, Any
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import requests
from bs4 import BeautifulSoup

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

def load_config() -> Dict[str, Any]:
    """Load configuration from environment variables"""
    load_dotenv()
    # Implementation here...
    pass

def get_all_urls(base_url: str) -> List[str]:
    """Crawl sitemap and return all URLs"""
    # Implementation here...
    pass

def extract_text_from_url(url: str) -> Dict[str, Any]:
    """Extract text and metadata from a page"""
    # Implementation here...
    pass

def chunk_text(content: str, ...) -> List[Dict[str, Any]]:
    """Split content into chunks"""
    # Implementation here...
    pass

def embed_chunks(chunks: List[Dict[str, Any]], ...) -> List[Dict[str, Any]]:
    """Generate embeddings using Cohere"""
    # Implementation here...
    pass

def create_collection(qdrant_client, collection_name: str, ...) -> None:
    """Create Qdrant collection"""
    # Implementation here...
    pass

def save_chunks_to_qdrant(qdrant_client, ...) -> None:
    """Save chunks to Qdrant"""
    # Implementation here...
    pass

def main() -> None:
    """Main pipeline execution"""
    logging.info("Starting RAG content ingestion...")

    # 1. Load config
    config = load_config()

    # 2. Initialize clients
    co = cohere.Client(api_key=config["cohere_api_key"])
    qd = QdrantClient(url=config["qdrant_url"], api_key=config["qdrant_api_key"])

    # 3. Create collection
    create_collection(qd, config["collection_name"], config["vector_size"])

    # 4. Crawl website
    urls = get_all_urls(config["base_url"])
    logging.info(f"Discovered {len(urls)} URLs")

    # 5. Extract content
    pages = [extract_text_from_url(url) for url in urls]

    # 6. Chunk content
    all_chunks = []
    for page in pages:
        chunks = chunk_text(
            page["content"],
            config["chunk_size"],
            config["chunk_overlap"],
            config["code_threshold"],
            page["url"],
            page["title"],
            page["section_hierarchy"]
        )
        all_chunks.extend(chunks)
    logging.info(f"Created {len(all_chunks)} chunks")

    # 7. Generate embeddings
    chunks_with_embeddings = embed_chunks(all_chunks, co, config["cohere_model"])

    # 8. Save to Qdrant
    save_chunks_to_qdrant(qd, config["collection_name"], chunks_with_embeddings)

    # 9. Verify with sample query
    # ... verification implementation ...

    logging.info("Ingestion complete!")

if __name__ == "__main__":
    main()
```

**Full implementation**: Follow specifications in `contracts/schemas.md`

---

## Step 6: Run the Ingestion Pipeline

### Activate Virtual Environment and Run

```bash
# Run with uv (automatically uses .venv)
uv run python main.py
```

### Expected Output

```
2025-12-21 10:00:00 - INFO - Starting RAG content ingestion...
2025-12-21 10:00:01 - INFO - Loaded configuration
2025-12-21 10:00:02 - INFO - Connected to Cohere API
2025-12-21 10:00:02 - INFO - Connected to Qdrant Cloud
2025-12-21 10:00:03 - INFO - Collection 'rag_embedding' created successfully
2025-12-21 10:00:05 - INFO - Discovered 142 URLs from sitemap
2025-12-21 10:00:10 - INFO - Processed 50/142 pages
2025-12-21 10:00:15 - INFO - Processed 100/142 pages
2025-12-21 10:00:18 - INFO - Processed 142/142 pages
2025-12-21 10:00:18 - INFO - Created 1834 chunks
2025-12-21 10:00:20 - INFO - Embedded 50/1834 chunks
2025-12-21 10:00:25 - INFO - Embedded 100/1834 chunks
...
2025-12-21 10:05:00 - INFO - Embedded 1834/1834 chunks
2025-12-21 10:05:05 - INFO - Saved 100/1834 chunks to rag_embedding
...
2025-12-21 10:05:30 - INFO - Successfully saved 1834 chunks
2025-12-21 10:05:31 - INFO - Verification query: "How do I get started?"
2025-12-21 10:05:32 - INFO - Top result: "Getting Started" (score: 0.91)
2025-12-21 10:05:32 - INFO - Ingestion complete!
```

**Processing time**: ~5-10 minutes for 150 pages (depends on page count and content length)

---

## Step 7: Verify Ingestion

### Check Qdrant Collection

```python
# Verify in Qdrant Cloud dashboard or via script

from qdrant_client import QdrantClient

client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your_api_key")

# Check collection info
info = client.get_collection("rag_embedding")
print(f"Total vectors: {info.points_count}")
print(f"Vector size: {info.config.params.vectors.size}")
print(f"Distance: {info.config.params.vectors.distance}")
```

Expected output:
```
Total vectors: 1834
Vector size: 1024
Distance: Cosine
```

### Test Similarity Search

```python
# Test search functionality

import cohere
from qdrant_client import QdrantClient

# Initialize clients
co = cohere.Client(api_key="your_cohere_key")
qd = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your_qdrant_key")

# Generate query embedding
query = "How do I authenticate API requests?"
query_embedding = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
).embeddings[0]

# Search Qdrant
results = qd.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=5
)

# Print results
for i, result in enumerate(results, 1):
    print(f"{i}. {result.payload['page_title']} (score: {result.score:.2f})")
    print(f"   {result.payload['page_url']}")
    print(f"   {result.payload['content'][:100]}...")
    print()
```

---

## Troubleshooting

### Issue: "Module 'cohere' not found"

**Solution**: Ensure you're running with `uv run`:
```bash
uv run python main.py
```

Or activate the virtual environment manually:
```bash
source .venv/bin/activate  # macOS/Linux
.venv\Scripts\activate  # Windows
python main.py
```

---

### Issue: "Cohere API Error 429: Too Many Requests"

**Solution**: You've hit the rate limit. Options:
1. Reduce `BATCH_SIZE` in `.env` (e.g., `BATCH_SIZE=25`)
2. Implement longer backoff delays in `embed_chunks()`
3. Upgrade to paid Cohere tier for higher limits

---

### Issue: "Qdrant Cloud connection timeout"

**Solution**:
1. Verify `QDRANT_URL` is correct (check Qdrant dashboard)
2. Ensure API key is valid and not expired
3. Check firewall/network settings

---

### Issue: "No URLs discovered from sitemap"

**Solution**:
1. Verify `BASE_URL` in `.env` is correct
2. Check if sitemap exists: visit `{BASE_URL}/sitemap.xml`
3. If no sitemap, modify `get_all_urls()` to use link crawling instead

---

### Issue: "Content extraction returns empty content"

**Solution**:
1. Check page structure with browser DevTools
2. Update BeautifulSoup selectors in `extract_text_from_url()`
3. Docusaurus versions may use different CSS classes

---

## Next Steps

After successful ingestion:

1. **Build a query interface**: Create a search API or UI
2. **Implement incremental updates**: Detect changed pages and re-ingest
3. **Add filters**: Search by section, content type, or page
4. **Monitor performance**: Track API usage and costs
5. **Scale up**: Process larger documentation sites (1000+ pages)

## Additional Resources

- **Cohere Documentation**: [https://docs.cohere.com/docs/embeddings](https://docs.cohere.com/docs/embeddings)
- **Qdrant Documentation**: [https://qdrant.tech/documentation](https://qdrant.tech/documentation)
- **UV Documentation**: [https://docs.astral.sh/uv](https://docs.astral.sh/uv)
- **BeautifulSoup Guide**: [https://www.crummy.com/software/BeautifulSoup/bs4/doc/](https://www.crummy.com/software/BeautifulSoup/bs4/doc/)

## Summary

You've successfully:
- ✅ Set up Python environment with `uv`
- ✅ Configured Cohere and Qdrant Cloud
- ✅ Implemented content ingestion pipeline
- ✅ Ingested documentation into vector database
- ✅ Verified semantic search functionality

**Your RAG system is ready for queries!**
