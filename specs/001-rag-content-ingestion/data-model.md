# Data Model: RAG Content Ingestion

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-21
**Purpose**: Define data structures for documentation pages, chunks, embeddings, and configurations

## Overview

The RAG content ingestion pipeline processes documentation pages through four main data transformations:

1. **DocumentationPage**: Raw page data extracted from Docusaurus website
2. **ContentChunk**: Segmented text optimized for embedding
3. **VectorEmbedding**: Embedded chunks ready for storage
4. **IngestionConfig**: Runtime configuration parameters

## Core Entities

### 1. DocumentationPage

Represents a single documentation page extracted from the Docusaurus website.

**Schema**:
```python
{
    "url": str,                      # Full URL of the page (e.g., "https://docs.example.com/guides/intro")
    "title": str,                    # Page title extracted from <h1> or <title>
    "content": str,                  # Full raw text content (cleaned HTML)
    "section_hierarchy": List[str],  # Breadcrumb path (e.g., ["Guides", "Getting Started", "Introduction"])
    "last_modified": Optional[str],  # ISO 8601 timestamp (if available from sitemap)
}
```

**Example**:
```python
{
    "url": "https://docs.example.com/api/authentication",
    "title": "Authentication",
    "content": "API authentication uses API keys...\n\n```python\nimport requests\n...",
    "section_hierarchy": ["API Reference", "Authentication"],
    "last_modified": "2025-12-20T10:30:00Z"
}
```

**Validation Rules**:
- `url` must be a valid HTTP/HTTPS URL
- `title` must be non-empty string
- `content` must be non-empty string
- `section_hierarchy` can be empty list if no breadcrumbs found

**Relationships**:
- One DocumentationPage yields multiple ContentChunks

---

### 2. ContentChunk

A segment of a documentation page optimized for embedding generation.

**Schema**:
```python
{
    "chunk_id": str,                 # UUID (generated via uuid.uuid4())
    "page_url": str,                 # Source page URL (reference to DocumentationPage)
    "page_title": str,               # Source page title
    "content": str,                  # Chunk text content (max ~1000 characters)
    "chunk_index": int,              # Position within page (0-based)
    "section_hierarchy": List[str],  # Inherited from parent page
    "heading": Optional[str],        # Nearest section heading (if detectable)
    "content_type": str,             # "prose" | "code" | "mixed"
    "metadata": dict,                # Extensible metadata (e.g., {"language": "python"})
}
```

**Example (Prose Chunk)**:
```python
{
    "chunk_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "page_url": "https://docs.example.com/api/authentication",
    "page_title": "Authentication",
    "content": "API authentication uses API keys. Each request must include an API key in the Authorization header. API keys are generated from the dashboard...",
    "chunk_index": 0,
    "section_hierarchy": ["API Reference", "Authentication"],
    "heading": "Overview",
    "content_type": "prose",
    "metadata": {}
}
```

**Example (Code Chunk)**:
```python
{
    "chunk_id": "b2c3d4e5-f6a7-8901-bcde-f12345678901",
    "page_url": "https://docs.example.com/api/authentication",
    "page_title": "Authentication",
    "content": "```python\nimport requests\n\nheaders = {\n    'Authorization': 'Bearer YOUR_API_KEY'\n}\n\nresponse = requests.get('https://api.example.com/data', headers=headers)\nprint(response.json())\n```",
    "chunk_index": 1,
    "section_hierarchy": ["API Reference", "Authentication"],
    "heading": "Example",
    "content_type": "code",
    "metadata": {"language": "python"}
}
```

**Validation Rules**:
- `chunk_id` must be unique UUID
- `content` must be non-empty and ≤1200 characters (buffer for overlap)
- `chunk_index` must be non-negative integer
- `content_type` must be one of: "prose", "code", "mixed"

**Content Type Detection**:
- **"prose"**: No code blocks detected (no ``` or <pre><code>)
- **"code"**: Entire chunk is a code block
- **"mixed"**: Contains both prose and code blocks

**Relationships**:
- Many ContentChunks belong to one DocumentationPage
- One ContentChunk yields one VectorEmbedding

---

### 3. VectorEmbedding

A numerical vector representation of a content chunk, ready for storage in Qdrant.

**Schema**:
```python
{
    "id": str,                       # Qdrant point ID (same as chunk_id)
    "vector": List[float],           # Embedding vector (1024 dimensions for Cohere embed-english-v3.0)
    "payload": {                     # Qdrant metadata (searchable)
        "page_url": str,
        "page_title": str,
        "content": str,              # Original chunk text (for display in results)
        "chunk_index": int,
        "section_hierarchy": List[str],
        "heading": Optional[str],
        "content_type": str,
    }
}
```

**Example**:
```python
{
    "id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "vector": [0.123, -0.456, 0.789, ...],  # 1024 floats
    "payload": {
        "page_url": "https://docs.example.com/api/authentication",
        "page_title": "Authentication",
        "content": "API authentication uses API keys. Each request must include...",
        "chunk_index": 0,
        "section_hierarchy": ["API Reference", "Authentication"],
        "heading": "Overview",
        "content_type": "prose",
    }
}
```

**Validation Rules**:
- `id` must match the chunk_id
- `vector` must have exactly 1024 dimensions
- All vector values must be floats (can be negative)
- `payload` must include all required fields from ContentChunk

**Relationships**:
- One VectorEmbedding corresponds to one ContentChunk
- Stored in Qdrant collection "rag_embedding"

---

### 4. IngestionConfig

Runtime configuration for the ingestion pipeline.

**Schema**:
```python
{
    "base_url": str,                 # Docusaurus site base URL (e.g., "https://docs.example.com")
    "chunk_size": int,               # Target chunk size in characters (default: 1000)
    "chunk_overlap": int,            # Overlap between chunks in characters (default: 200)
    "code_threshold": int,           # Max code block size before split (default: 500)
    "cohere_model": str,             # Cohere embedding model name (default: "embed-english-v3.0")
    "cohere_input_type": str,        # Input type for embeddings (default: "search_document")
    "qdrant_collection": str,        # Collection name (default: "rag_embedding")
    "qdrant_distance": str,          # Distance metric (default: "Cosine")
    "vector_size": int,              # Vector dimensions (default: 1024)
    "batch_size": int,               # Embedding batch size (default: 50, max: 96)
}
```

**Example**:
```python
{
    "base_url": "https://docs.example.com",
    "chunk_size": 1000,
    "chunk_overlap": 200,
    "code_threshold": 500,
    "cohere_model": "embed-english-v3.0",
    "cohere_input_type": "search_document",
    "qdrant_collection": "rag_embedding",
    "qdrant_distance": "Cosine",
    "vector_size": 1024,
    "batch_size": 50,
}
```

**Validation Rules**:
- `base_url` must be valid HTTP/HTTPS URL
- `chunk_size` must be > 0 and ≤ 2000
- `chunk_overlap` must be ≥ 0 and < chunk_size
- `code_threshold` must be > 0 and < chunk_size
- `cohere_model` must be valid Cohere model name
- `qdrant_collection` must be alphanumeric + underscores
- `vector_size` must match Cohere model output (1024 for v3.0)
- `batch_size` must be > 0 and ≤ 96 (Cohere limit)

**Source**: Loaded from environment variables with defaults

---

## Data Flow

```
1. Crawl Website
   └─> List[str] (URLs)

2. Extract Content
   └─> List[DocumentationPage]

3. Chunk Content
   └─> List[ContentChunk]

4. Generate Embeddings
   └─> List[VectorEmbedding]

5. Store in Qdrant
   └─> Success/Failure status
```

## Qdrant Collection Schema

**Collection Name**: `rag_embedding`

**Vector Configuration**:
```python
from qdrant_client.models import Distance, VectorParams

vector_params = VectorParams(
    size=1024,
    distance=Distance.COSINE,
)
```

**Payload Indexing** (for filtering):
```python
# Enable filtering on these fields
payload_indexes = {
    "page_url": "keyword",
    "content_type": "keyword",
    "section_hierarchy": "keyword",
}
```

**Example Query** (similarity search with filter):
```python
results = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=5,
    query_filter={
        "must": [
            {"key": "content_type", "match": {"value": "prose"}}
        ]
    }
)
```

## Data Validation

All entities should be validated before processing:

```python
from typing import Dict, Any, List

def validate_documentation_page(page: Dict[str, Any]) -> None:
    assert "url" in page and page["url"].startswith("http")
    assert "title" in page and page["title"].strip()
    assert "content" in page and page["content"].strip()
    assert "section_hierarchy" in page and isinstance(page["section_hierarchy"], list)

def validate_content_chunk(chunk: Dict[str, Any]) -> None:
    assert "chunk_id" in chunk
    assert "content" in chunk and 0 < len(chunk["content"]) <= 1200
    assert "chunk_index" in chunk and chunk["chunk_index"] >= 0
    assert chunk["content_type"] in ["prose", "code", "mixed"]

def validate_vector_embedding(embedding: Dict[str, Any]) -> None:
    assert "id" in embedding
    assert "vector" in embedding and len(embedding["vector"]) == 1024
    assert "payload" in embedding
    validate_content_chunk(embedding["payload"])  # Payload is ContentChunk
```

## State Transitions

```
Raw HTML (Website)
   ↓ extract_text_from_url()
DocumentationPage
   ↓ chunk_text()
List[ContentChunk]
   ↓ embed_chunks()
List[ContentChunk + embedding]
   ↓ save_chunks_to_qdrant()
Qdrant Storage (VectorEmbedding)
```

## Error Handling

**Invalid Data**:
- Empty content: Skip page/chunk, log warning
- Malformed HTML: Try fallback selectors, skip if fails
- Missing fields: Fill with defaults or skip

**Data Constraints**:
- Content too long: Split into multiple chunks
- Invalid URLs: Skip page, log error
- Duplicate chunks: Use content hash to detect, skip duplicates

## Next Steps

1. Implement data structures in `backend/main.py`
2. Add validation functions for each entity
3. Test with sample Docusaurus page
4. Verify Qdrant storage and retrieval
