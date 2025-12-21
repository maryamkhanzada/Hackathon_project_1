# Research: RAG Content Ingestion Technologies

**Date**: 2025-12-21
**Feature**: 001-rag-content-ingestion
**Purpose**: Resolve technical unknowns and establish technology choices for implementation

## 1. Cohere Embedding API

**Decision**: Use `embed-english-v3.0` model with 1024-dimensional vectors

**Rationale**:
- High-quality embeddings optimized for English technical content
- 1024 dimensions provide excellent semantic representation
- Reasonable cost on free tier (100 API calls/minute, 1000 calls/month for trial)
- Input type flexibility (`search_document` for ingestion, `search_query` for retrieval)
- Supports batch embedding (up to 96 texts per request)

**Alternatives Considered**:
- `embed-english-light-v3.0` (768 dims): Smaller vectors, faster but lower quality
- `embed-multilingual-v3.0`: Not needed for English-only documentation
- OpenAI `text-embedding-3-small`: More expensive, similar quality

**Configuration**:
```python
model = "embed-english-v3.0"
input_type = "search_document"  # For document ingestion
dimensions = 1024  # Default output dimension
truncate = "END"  # Truncate long inputs from the end
```

**API Limits**:
- Free tier (trial): 100 calls/min, 1000 calls/month
- Paid tier: Higher limits based on plan
- Max input length: ~512 tokens per text
- Batch size: Up to 96 texts per request

**Rate Limiting Strategy**:
- Batch chunks (up to 96 per request)
- Implement exponential backoff on 429 errors
- Track API call count

## 2. Qdrant Cloud Setup

**Decision**: Use Qdrant Cloud with Cosine distance metric and 1024-dimensional vectors

**Rationale**:
- Managed cloud service eliminates infrastructure setup
- Free tier supports up to 1GB storage (~1M vectors)
- Cosine similarity is standard for normalized embeddings
- Excellent performance for semantic search (<200ms)
- Simple Python client integration

**Alternatives Considered**:
- Self-hosted Qdrant: Requires server management, more complex
- Pinecone: More expensive, similar features
- Weaviate Cloud: Comparable but less familiar

**Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

# Connection
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),  # https://xxx.qdrant.io
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Collection config
collection_name = "rag_embedding"
vector_params = VectorParams(
    size=1024,  # Match Cohere dimensions
    distance=Distance.COSINE,
)
```

**Free Tier Limits**:
- Storage: 1GB (~1M vectors of 1024 dims)
- Requests: Unlimited reads, writes
- Expected capacity: 5000 chunks @ 1024 dims ≈ 20MB (well under limit)

**Metadata Schema**:
```python
payload = {
    "page_url": str,
    "page_title": str,
    "content": str,  # Original chunk text
    "chunk_index": int,
    "section_hierarchy": List[str],
    "heading": Optional[str],
    "content_type": str,  # "prose" | "code" | "mixed"
}
```

## 3. Docusaurus Crawling

**Decision**: Sitemap-based crawling with BeautifulSoup content extraction

**Rationale**:
- Docusaurus generates `sitemap.xml` with all pages
- Sitemap provides complete, authoritative page list
- BeautifulSoup handles HTML parsing efficiently
- CSS selectors target main content area (`article`, `.markdown`)

**Alternatives Considered**:
- Link following (BFS/DFS): Slower, may miss pages
- Scrapy framework: Overkill for single-site crawling
- Selenium for JS rendering: Docusaurus serves static HTML

**Implementation**:

### URL Discovery
```python
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin

def get_all_urls(base_url: str) -> List[str]:
    """Discover all URLs from sitemap.xml"""
    sitemap_url = urljoin(base_url, "/sitemap.xml")
    response = requests.get(sitemap_url)
    soup = BeautifulSoup(response.content, "xml")
    urls = [loc.text for loc in soup.find_all("loc")]
    return urls
```

### Content Extraction
```python
def extract_text_from_url(url: str) -> Dict[str, Any]:
    """Extract main content, title, and hierarchy"""
    response = requests.get(url)
    soup = BeautifulSoup(response.content, "html.parser")

    # Title extraction
    title = soup.find("h1") or soup.find("title")
    title_text = title.get_text(strip=True) if title else "Untitled"

    # Main content (Docusaurus-specific selectors)
    main_content = soup.find("article") or soup.find("main")
    if not main_content:
        main_content = soup.find(class_="markdown")

    # Extract text while preserving structure
    content = main_content.get_text(separator="\n", strip=True)

    # Breadcrumb/hierarchy extraction
    breadcrumb = soup.find(class_="breadcrumbs")
    hierarchy = []
    if breadcrumb:
        hierarchy = [a.get_text(strip=True) for a in breadcrumb.find_all("a")]

    return {
        "url": url,
        "title": title_text,
        "content": content,
        "section_hierarchy": hierarchy,
    }
```

**Docusaurus-Specific Selectors**:
- Main content: `<article>`, `<main>`, `.markdown`
- Title: `<h1>` or `<title>`
- Breadcrumbs: `.breadcrumbs`
- Code blocks: `<pre><code>` (preserved in text extraction)

## 4. Chunking Strategy

**Decision**: Adaptive character-based chunking with code block awareness

**Rationale**:
- Character-based is simpler than token-based (no tokenizer needed)
- 1000 characters ≈ 200-300 tokens (safe for Cohere 512 token limit)
- 200 character overlap preserves context between chunks
- Small code blocks (<500 chars) stay with surrounding text
- Large code blocks (≥500 chars) split separately

**Alternatives Considered**:
- Fixed chunking (no code awareness): Breaks code blocks awkwardly
- Token-based chunking: Requires tokenizer, more complex
- Semantic chunking (sentence/paragraph boundaries): More complex, slower

**Parameters**:
```python
CHUNK_SIZE = 1000  # characters
CHUNK_OVERLAP = 200  # characters
CODE_THRESHOLD = 500  # characters (split if code block exceeds this)
```

**Implementation Logic**:
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
    Adaptive chunking:
    1. Detect code blocks (markdown ``` or HTML <pre><code>)
    2. If code block < CODE_THRESHOLD: keep with surrounding text
    3. If code block >= CODE_THRESHOLD: split as separate chunk
    4. For remaining text: sliding window with overlap
    """
    chunks = []
    # ... implementation ...
    return chunks
```

**Content Type Detection**:
- Prose: No code blocks detected
- Code: Entire chunk is code
- Mixed: Contains both prose and code

## 5. UV Workflow

**Decision**: Use `uv` for all Python environment and dependency management

**Rationale**:
- Modern, fast alternative to pip/virtualenv
- Automatic virtual environment creation
- Lockfile support for reproducibility
- Simple commands for adding dependencies
- Better resolution of dependency conflicts

**Alternatives Considered**:
- pip + virtualenv: Older, slower, more manual
- Poetry: Good but heavier, unnecessary for simple project
- Conda: Overkill for Python-only project

**Setup Steps**:

### 1. Install UV
```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 2. Initialize Project
```bash
mkdir backend
cd backend
uv init  # Creates pyproject.toml
uv python pin 3.11  # Sets Python version
```

### 3. Add Dependencies
```bash
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv lxml
```

This creates:
- `pyproject.toml` (project metadata and dependencies)
- `uv.lock` (locked dependency versions)
- `.venv/` (virtual environment)

### 4. Sync Environment
```bash
uv sync  # Install all dependencies from lockfile
```

### 5. Run Scripts
```bash
uv run python main.py  # Runs in virtual environment
```

**Generated `pyproject.toml`**:
```toml
[project]
name = "rag-content-ingestion"
version = "0.1.0"
description = "RAG content ingestion pipeline for Docusaurus documentation"
requires-python = ">=3.11"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "python-dotenv>=1.0.0",
    "lxml>=5.0.0",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

## Summary of Resolved Clarifications

| Research Area | Clarification Needed | Decision |
|---------------|---------------------|----------|
| Cohere Model | Which model and dimensions? | embed-english-v3.0, 1024-dim |
| Cohere Limits | Rate limits and handling? | 100/min free tier, batch + backoff |
| Qdrant Config | Distance metric and setup? | Cosine distance, cloud managed |
| Qdrant Limits | Free tier capacity? | 1GB storage, sufficient for 5K chunks |
| Crawling | Sitemap vs link following? | Sitemap.xml for complete coverage |
| Content Extraction | Which CSS selectors? | `<article>`, `<main>`, `.markdown` |
| Chunking | Character vs token-based? | Character-based (simpler, faster) |
| Code Handling | When to split code blocks? | Split if ≥500 chars, else keep with text |
| UV Commands | How to use uv? | init, add, sync, run |
| Python Version | Which version? | 3.11+ |

## Next Steps

All technical unknowns resolved. Proceed to Phase 1:
1. Generate data-model.md (entity schemas)
2. Create contracts/schemas.md (function signatures)
3. Write quickstart.md (setup guide)
4. Update agent context with new technologies
