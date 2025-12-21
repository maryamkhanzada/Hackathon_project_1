# Implementation Plan: RAG Content Ingestion System

**Branch**: `001-rag-content-ingestion` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-content-ingestion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build an automated content ingestion pipeline that extracts documentation from a deployed Docusaurus website, generates semantic embeddings using Cohere, and stores them in Qdrant Cloud for similarity search. The system will be implemented as a single Python script (`backend/main.py`) using the `uv` package manager for dependency management and virtual environment setup.

**Key Technical Approach**:
- Use `uv` for Python project initialization and dependency management
- Implement all functionality in a single `backend/main.py` file with modular functions
- Use Cohere API for generating embeddings (instead of sentence-transformers)
- Store vectors in Qdrant Cloud (managed cloud service)
- Implement configurable text chunking with metadata preservation
- Verify ingestion with a similarity search query

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- `cohere` (embedding generation via Cohere API)
- `qdrant-client` (Qdrant Cloud integration)
- `requests` (HTTP requests for web scraping)
- `beautifulsoup4` (HTML parsing)
- `python-dotenv` (environment variable management)

**Storage**: Qdrant Cloud (managed vector database service)
**Testing**: Manual verification via similarity search query (pytest for future iterations)
**Target Platform**: Local development / server environment with internet access
**Project Type**: Single Python script (backend/)
**Performance Goals**:
- Extract and process 500+ pages within 1 hour
- Embedding API calls with rate limiting handling
- Chunk processing speed optimized for sequential execution

**Constraints**:
- Single file implementation (main.py)
- Cohere API rate limits (managed with retry logic)
- Qdrant Cloud free tier limits (if applicable)
- Network dependency for both Cohere API and Qdrant Cloud

**Scale/Scope**:
- Target: 500+ documentation pages
- Estimated chunks: 2000-5000 depending on page length
- Vector dimensions: 1024 (Cohere embed-english-v3.0 default) or 768 (embed-english-light-v3.0)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution Principles

✅ **Spec-driven writing**: Complete spec.md exists with all requirements documented
✅ **Technical accuracy**: All dependencies and APIs will be verified during implementation
✅ **Clarity**: Single-file design makes code accessible and understandable
✅ **Reproducibility**: Environment setup via `uv` ensures consistent dependencies
✅ **Tool-first workflow**: Using Spec-Kit Plus for planning and task management

### Standards Compliance

✅ **Verified commands**: All `uv` commands and API calls will be tested before documentation
✅ **Consistent versions**: Pinned dependencies in `pyproject.toml` (uv-managed)
✅ **Copy-paste ready**: Environment setup and execution steps will be fully documented
✅ **Proper structure**: `backend/` directory with clear organization
✅ **No hallucinations**: Using official Cohere and Qdrant APIs as documented

### Constraints

⚠️ **Build requirement**: N/A (not a Docusaurus project)
⚠️ **GitHub Pages deployment**: N/A (backend ingestion script)
✅ **Original content**: All code will be custom-written for this specific use case
✅ **Zero errors**: Error handling for network failures, API errors, and parsing issues

**Status**: ✅ PASSED - Single Python script design aligns with simplicity and reproducibility principles

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-content-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (dependency research, API analysis)
├── data-model.md        # Phase 1 output (chunk structure, metadata schema)
├── quickstart.md        # Phase 1 output (setup and execution guide)
├── contracts/           # Phase 1 output (API contracts, data schemas)
│   └── schemas.md       # JSON schemas for chunks, metadata, configs
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Single file with all ingestion logic
├── .env.example         # Template for environment variables
├── .env                 # Actual secrets (gitignored)
├── pyproject.toml       # uv project configuration
└── .python-version      # Python version for uv

.gitignore               # Updated to include backend/.env, .venv/
```

**Structure Decision**: Single project structure selected because the entire ingestion pipeline is implemented in one Python script (`backend/main.py`). This maximizes simplicity and aligns with the user's request for a single-file implementation. The `uv` tool will manage the virtual environment and dependencies via `pyproject.toml`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. The single-file design is the simplest possible approach for this feature.

## Phase 0: Research & Technology Validation

### Research Tasks

#### 1. Cohere Embedding API Research
**Objective**: Understand Cohere embedding models, dimensions, rate limits, and best practices

**Research Questions**:
- Which Cohere embedding model should be used? (embed-english-v3.0 vs embed-english-light-v3.0)
- What are the vector dimensions for each model?
- What are the rate limits and pricing for Cohere API?
- How to handle long text inputs (chunking requirements)?
- What are the input_type options for embedding generation?

**Decision Criteria**:
- Model quality vs. cost trade-off
- Vector dimensions compatible with Qdrant
- Rate limit handling strategy

#### 2. Qdrant Cloud Integration Research
**Objective**: Understand Qdrant Cloud setup, collection creation, and vector storage patterns

**Research Questions**:
- How to connect to Qdrant Cloud (API key authentication)?
- How to create collections with custom vector dimensions?
- What distance metrics are available (Cosine, Dot Product, Euclidean)?
- How to store metadata (payload) with vectors?
- What are the free tier limits?
- How to perform similarity search with filtering?

**Decision Criteria**:
- Collection configuration (distance metric, vector size)
- Metadata schema design
- Free tier capacity planning

#### 3. Docusaurus Website Crawling Research
**Objective**: Best practices for crawling Docusaurus sites and extracting clean text

**Research Questions**:
- How to discover all pages from a Docusaurus sitemap or navigation?
- How to extract main content area (exclude headers, footers, navigation)?
- How to preserve code blocks and markdown structure?
- How to extract page titles and section hierarchies?
- How to handle client-side rendered content (if any)?

**Decision Criteria**:
- HTML parsing strategy (BeautifulSoup selectors)
- Content cleaning approach
- Metadata extraction patterns

#### 4. Text Chunking Strategy Research
**Objective**: Implement adaptive chunking with configurable size/overlap for code blocks

**Research Questions**:
- What chunk size and overlap parameters work best for technical documentation?
- How to detect code blocks vs. prose?
- How to split large code blocks while preserving context?
- How to preserve section headings with chunks?

**Decision Criteria**:
- Default chunk size (tokens or characters)
- Overlap percentage
- Code block handling logic

#### 5. UV Package Manager Research
**Objective**: Understand `uv` for project initialization, virtual env, and dependency management

**Research Questions**:
- How to initialize a Python project with `uv init`?
- How to create and activate virtual environments with `uv`?
- How to add dependencies with `uv add`?
- How to sync dependencies with `uv sync`?
- How to run Python scripts in the `uv` environment?

**Decision Criteria**:
- Project initialization workflow
- Dependency pinning strategy
- Execution commands

### Research Output: research.md

**Location**: `specs/001-rag-content-ingestion/research.md`

**Structure**:
```markdown
# Research: RAG Content Ingestion Technologies

## 1. Cohere Embedding API
**Decision**: [chosen model]
**Rationale**: [quality, cost, dimensions]
**Alternatives**: [other models considered]
**Configuration**: [model name, input_type, dimensions]

## 2. Qdrant Cloud Setup
**Decision**: [collection config]
**Rationale**: [distance metric choice, metadata schema]
**Alternatives**: [other vector DBs, self-hosted Qdrant]
**Configuration**: [collection name, vector size, distance metric]

## 3. Docusaurus Crawling
**Decision**: [crawling approach]
**Rationale**: [sitemap vs. link following]
**Alternatives**: [other scraping methods]
**Implementation**: [BeautifulSoup selectors, content extraction]

## 4. Chunking Strategy
**Decision**: [chunk size, overlap, code handling]
**Rationale**: [balance between context and granularity]
**Alternatives**: [fixed vs. adaptive chunking]
**Parameters**: [chunk_size, chunk_overlap, code_threshold]

## 5. UV Workflow
**Decision**: [uv commands and workflow]
**Rationale**: [modern Python tooling, speed]
**Setup Steps**: [init, add deps, sync, run]
```

## Phase 1: Design & Contracts

### 1. Data Model Design (data-model.md)

**Location**: `specs/001-rag-content-ingestion/data-model.md`

**Entities** (from spec.md):

#### DocumentationPage
```python
{
    "url": str,              # Full URL of the page
    "title": str,            # Page title
    "content": str,          # Full raw content
    "section_hierarchy": List[str],  # Breadcrumb path
    "last_modified": Optional[str],  # ISO timestamp if available
}
```

#### ContentChunk
```python
{
    "chunk_id": str,         # UUID for the chunk
    "page_url": str,         # Source page URL
    "page_title": str,       # Source page title
    "content": str,          # Chunk text content
    "chunk_index": int,      # Position within page (0-based)
    "section_hierarchy": List[str],  # Breadcrumb context
    "heading": Optional[str],  # Nearest heading
    "content_type": str,     # "prose" | "code" | "mixed"
    "metadata": dict,        # Additional metadata
}
```

#### VectorEmbedding
```python
{
    "id": str,               # Same as chunk_id (Qdrant point ID)
    "vector": List[float],   # Embedding vector (1024 or 768 dims)
    "payload": {             # Qdrant metadata
        "page_url": str,
        "page_title": str,
        "content": str,
        "chunk_index": int,
        "section_hierarchy": List[str],
        "heading": Optional[str],
        "content_type": str,
    }
}
```

#### IngestionConfig
```python
{
    "base_url": str,         # Docusaurus site base URL
    "chunk_size": int,       # Characters per chunk
    "chunk_overlap": int,    # Character overlap
    "code_threshold": int,   # Max code block size before split
    "cohere_model": str,     # Cohere embedding model name
    "qdrant_collection": str,  # Collection name ("rag_embedding")
}
```

### 2. API Contracts (contracts/)

**Location**: `specs/001-rag-content-ingestion/contracts/schemas.md`

**Function Signatures** (main.py modules):

```python
# Web Crawling
def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site and return all documentation page URLs.

    Args:
        base_url: Base URL of the Docusaurus site

    Returns:
        List of absolute URLs for all discovered pages

    Raises:
        requests.RequestException: If base URL is unreachable
    """
    pass

# Content Extraction
def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extract clean text and metadata from a documentation page.

    Args:
        url: Absolute URL of the page

    Returns:
        {
            "url": str,
            "title": str,
            "content": str,
            "section_hierarchy": List[str],
        }

    Raises:
        requests.RequestException: If page is unreachable
        ValueError: If content cannot be extracted
    """
    pass

# Chunking
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
        code_threshold: Max code block size before splitting
        page_url: Source page URL
        page_title: Source page title
        section_hierarchy: Breadcrumb context

    Returns:
        List of ContentChunk dictionaries
    """
    pass

# Embedding Generation
def embed_chunks(chunks: List[Dict[str, Any]], cohere_client, model: str) -> List[Dict[str, Any]]:
    """
    Generate embeddings for chunks using Cohere API.

    Args:
        chunks: List of ContentChunk dictionaries
        cohere_client: Initialized Cohere client
        model: Cohere model name

    Returns:
        List of chunks with added "embedding" field

    Raises:
        cohere.CohereAPIError: If API request fails
    """
    pass

# Qdrant Operations
def create_collection(
    qdrant_client,
    collection_name: str,
    vector_size: int,
    distance: str = "Cosine"
) -> None:
    """
    Create Qdrant collection if it doesn't exist.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name for the collection
        vector_size: Dimension of vectors
        distance: Distance metric (Cosine, Dot, Euclidean)

    Raises:
        Exception: If collection creation fails
    """
    pass

def save_chunks_to_qdrant(
    qdrant_client,
    collection_name: str,
    chunks_with_embeddings: List[Dict[str, Any]]
) -> None:
    """
    Save embedded chunks to Qdrant collection.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Target collection name
        chunks_with_embeddings: Chunks with "embedding" field

    Raises:
        Exception: If upsert operation fails
    """
    pass

# Main Orchestration
def main() -> None:
    """
    Main execution function that orchestrates the entire pipeline.

    Steps:
        1. Load environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
        2. Initialize clients (Cohere, Qdrant)
        3. Create Qdrant collection
        4. Crawl website to get all URLs
        5. Extract content from each URL
        6. Chunk all content
        7. Generate embeddings
        8. Save to Qdrant
        9. Perform sample similarity search to verify
    """
    pass
```

### 3. Quickstart Guide (quickstart.md)

**Location**: `specs/001-rag-content-ingestion/quickstart.md`

**Content Structure**:
```markdown
# Quickstart: RAG Content Ingestion

## Prerequisites
- Python 3.11+
- Cohere API key (free tier: https://cohere.com)
- Qdrant Cloud account and API key (free tier: https://qdrant.tech/cloud)

## Setup

### 1. Install UV
[Instructions for installing uv]

### 2. Initialize Project
```bash
cd backend
uv init
uv python pin 3.11
```

### 3. Add Dependencies
```bash
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv
```

### 4. Configure Environment
Create `backend/.env`:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
BASE_URL=https://your-docusaurus-site.com
```

### 5. Run Ingestion
```bash
uv run python main.py
```

## Verification
The script will output:
- Number of URLs discovered
- Number of chunks created
- Number of embeddings generated
- Sample similarity search results

## Troubleshooting
[Common issues and solutions]
```

### 4. Agent Context Update

**Action**: Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`

**Expected Updates**:
- Add technologies: Python, uv, Cohere, Qdrant Cloud, BeautifulSoup
- Preserve existing book project context
- Update between technology stack markers

## Phase 2: Task Generation

**Note**: Phase 2 (task breakdown) is handled by the `/sp.tasks` command and is NOT part of this `/sp.plan` output.

The task generation will create `specs/001-rag-content-ingestion/tasks.md` with testable implementation tasks based on this plan.

## Architecture Decisions

### AD-001: Single File Implementation
**Decision**: Implement all functionality in `backend/main.py`
**Rationale**:
- User explicitly requested single-file design
- Simplifies deployment and understanding
- Modular functions provide logical separation
- Easy to refactor later if needed

**Trade-offs**:
- ✅ Simple to understand and deploy
- ✅ No import complexity
- ❌ Harder to unit test individual functions
- ❌ File will be ~300-500 lines

**Alternatives Rejected**:
- Multi-file structure (models/, services/, utils/) - More complex than needed for initial version

### AD-002: Cohere for Embeddings (Override)
**Decision**: Use Cohere API instead of sentence-transformers
**Rationale**:
- User explicitly specified Cohere
- Cloud-based, no local model management
- High-quality embeddings
- Simple API integration

**Trade-offs**:
- ✅ No model download or GPU requirements
- ✅ Managed service with versioning
- ❌ API costs (free tier limited)
- ❌ Network dependency
- ❌ Rate limits

**Alternatives Rejected**:
- sentence-transformers (from spec) - User requested Cohere instead
- OpenAI embeddings - More expensive

### AD-003: Qdrant Cloud
**Decision**: Use Qdrant Cloud instead of self-hosted
**Rationale**:
- User specified Qdrant Cloud
- Managed service, no infrastructure
- Free tier available
- Production-ready

**Trade-offs**:
- ✅ No server setup
- ✅ Automatic scaling
- ❌ Network dependency
- ❌ Free tier limits

### AD-004: UV Package Manager
**Decision**: Use `uv` for dependency management
**Rationale**:
- User explicitly requested `uv`
- Modern, fast Python tooling
- Simple project initialization
- Better than pip/virtualenv

**Trade-offs**:
- ✅ Fast dependency resolution
- ✅ Lockfile support
- ✅ Simple commands
- ❌ Requires users to install `uv`

### AD-005: Collection Name
**Decision**: Use "rag_embedding" as collection name
**Rationale**: User explicitly specified this name

### AD-006: Synchronous Processing
**Decision**: Sequential URL processing (no async/parallel)
**Rationale**:
- Simpler implementation
- Avoids rate limiting complexity
- Acceptable for initial version

**Trade-offs**:
- ✅ Simple to implement and debug
- ❌ Slower for large documentation sets
- ❌ Not optimal for 500+ pages

**Future Enhancement**: Add async crawling and batch embedding

## Risk Analysis

### Risk 1: Cohere API Rate Limits
**Likelihood**: Medium
**Impact**: High (blocks ingestion)
**Mitigation**:
- Implement retry logic with exponential backoff
- Batch embed requests (Cohere supports batch)
- Monitor rate limit headers

### Risk 2: Qdrant Cloud Free Tier Limits
**Likelihood**: Medium
**Impact**: Medium (limits storage)
**Mitigation**:
- Calculate expected vector count before ingestion
- Provide clear error if limit exceeded
- Document paid tier upgrade path

### Risk 3: Docusaurus Structure Variations
**Likelihood**: High
**Impact**: Medium (incomplete crawling)
**Mitigation**:
- Test with multiple Docusaurus versions
- Provide configurable CSS selectors
- Fallback to generic HTML parsing

### Risk 4: Large Code Blocks
**Likelihood**: Medium
**Impact**: Low (chunking quality)
**Mitigation**:
- Implement adaptive splitting for code blocks
- Preserve code block integrity where possible
- Test with real documentation examples

## Implementation Notes

### Function Implementation Order
1. `get_all_urls` - Crawl and discover URLs
2. `extract_text_from_url` - Parse HTML and extract content
3. `chunk_text` - Split content with adaptive logic
4. `create_collection` - Initialize Qdrant collection
5. `embed_chunks` - Generate embeddings via Cohere
6. `save_chunks_to_qdrant` - Store vectors
7. `main` - Orchestrate pipeline and verify

### Configuration Management
All configuration will be in `.env`:
```env
# Required
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
BASE_URL=...

# Optional (with defaults)
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
CODE_THRESHOLD=500
COHERE_MODEL=embed-english-v3.0
COLLECTION_NAME=rag_embedding
```

### Error Handling Strategy
- Network errors: Retry with exponential backoff (max 3 attempts)
- Parsing errors: Log and skip page, continue processing
- API errors: Surface immediately, fail fast
- Validation errors: Fail fast with clear messages

### Logging Strategy
Use Python `logging` module:
- INFO: Progress updates (URLs discovered, chunks created, embeddings generated)
- WARNING: Recoverable errors (page skipped, retry attempt)
- ERROR: Fatal errors (API failure, missing credentials)

## Success Criteria Mapping

| Success Criterion | Implementation Approach |
|-------------------|-------------------------|
| SC-001: 100% page extraction | `get_all_urls` discovers all sitemap/linked pages |
| SC-002: 500+ pages in <1hr | Sequential processing (measured in testing) |
| SC-003: 90%+ search satisfaction | Quality chunking + Cohere embeddings |
| SC-005: <200ms similarity search | Qdrant Cloud performance (inherent) |
| SC-007: 95%+ chunk coherence | Adaptive chunking with context preservation |
| SC-008: 80% dedup reduction | Hash-based duplicate detection in chunking |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks from this plan
2. Execute tasks in priority order (P1 → P2 → P3)
3. Test each component incrementally
4. Verify end-to-end pipeline with sample Docusaurus site
5. Document any deviations or learnings in ADRs
