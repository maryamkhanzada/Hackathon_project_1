# Implementation Tasks: RAG Content Ingestion System

**Feature**: 001-rag-content-ingestion
**Branch**: `001-rag-content-ingestion`
**Date**: 2025-12-21
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Summary

Implement an automated content ingestion pipeline that extracts documentation from Docusaurus websites, generates embeddings using Cohere, and stores them in Qdrant Cloud. The system is implemented as a single Python script (`backend/main.py`) with modular functions organized by user story priorities.

**User Stories** (from spec.md):
- **US1 (P1)**: Automated Website Content Extraction - Foundation for all functionality
- **US2 (P2)**: Semantic Embedding Generation - Core RAG capability
- **US3 (P3)**: Vector Database Storage and Indexing - Complete the pipeline
- **US4 (P4)**: Incremental Updates and Change Detection - Optimization (out of scope for MVP)

**MVP Scope**: User Stories 1, 2, and 3 (US4 is future enhancement)

**Tech Stack**: Python 3.11+, uv, Cohere API, Qdrant Cloud, BeautifulSoup, requests

---

## Implementation Strategy

### Incremental Delivery

Each user story can be implemented and tested independently:

1. **US1 First**: Implement crawling and extraction → Test by verifying extracted content
2. **US2 Next**: Add chunking and embedding → Test by verifying chunk creation and embeddings
3. **US3 Last**: Add Qdrant storage → Test with similarity search queries
4. **US4**: Future iteration (not in MVP)

### Parallel Opportunities

Tasks marked with `[P]` can be executed in parallel if working on the same user story.

---

## Dependencies

### User Story Completion Order

```
Setup Phase (Phase 1)
    ↓
Foundational Phase (Phase 2)
    ↓
US1: Content Extraction (Phase 3) ← MUST complete first
    ↓
US2: Embedding Generation (Phase 4) ← Depends on US1 (chunks)
    ↓
US3: Vector Storage (Phase 5) ← Depends on US2 (embeddings)
    ↓
Polish Phase (Phase 6)
```

**Note**: US4 (Incremental Updates) is intentionally excluded from MVP scope as it's an optimization.

---

## Phase 1: Setup

**Goal**: Initialize project structure, dependencies, and environment configuration

**Tasks**:

- [x] T001 Create backend/ directory structure at repository root
- [x] T002 Initialize uv project in backend/ with `uv init`
- [x] T003 Pin Python version to 3.11 with `uv python pin 3.11`
- [x] T004 Add dependencies via `uv add cohere qdrant-client requests beautifulsoup4 python-dotenv lxml`
- [x] T005 Create backend/.env.example with all required environment variables
- [x] T006 Create backend/.env template (gitignored) with placeholder values
- [x] T007 Update .gitignore to exclude backend/.env and backend/.venv/
- [x] T008 Verify uv sync completes successfully and .venv/ is created

**Deliverables**:
- `backend/` directory with uv project initialized
- `pyproject.toml` with all dependencies
- `.env.example` and `.env` files
- Updated `.gitignore`

**Validation**:
```bash
cd backend
uv sync
uv run python --version  # Should show Python 3.11+
```

---

## Phase 2: Foundational

**Goal**: Implement shared utilities and configuration loading (blocking prerequisites)

**Tasks**:

- [ ] T009 Create backend/main.py with basic structure (imports, logging setup, main() function)
- [ ] T010 Implement load_config() function in backend/main.py to load environment variables with validation
- [ ] T011 Add error handling for missing required environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, BASE_URL)
- [ ] T012 Implement retry logic helper function with exponential backoff in backend/main.py
- [ ] T013 Set up logging configuration (INFO level, timestamp format) in backend/main.py

**Deliverables**:
- `backend/main.py` with configuration and utilities

**Independent Test Criteria**:
- Running `uv run python main.py` loads config without errors
- Missing env vars trigger clear error messages
- Logging outputs are formatted correctly

**Validation**:
```bash
# Test with missing env vars (should error)
uv run python main.py

# Test with valid .env (should load config)
# (Add actual credentials to .env first)
uv run python main.py
```

---

## Phase 3: User Story 1 - Automated Website Content Extraction (P1)

**Story Goal**: Extract all published content from a Docusaurus website with metadata and hierarchical structure preserved.

**Independent Test Criteria** (from spec.md):
- Point system at a Docusaurus site → all pages discovered and content extracted
- Markdown formatting, code blocks, metadata preserved
- Hierarchical structure and navigation context captured

**Tasks**:

- [x] T014 [P] [US1] Implement get_all_urls() function in backend/main.py to fetch sitemap.xml and extract URLs
- [x] T015 [P] [US1] Add URL filtering logic in get_all_urls() to exclude non-documentation pages (/blog, /tags)
- [x] T016 [P] [US1] Implement extract_text_from_url() function in backend/main.py to parse HTML with BeautifulSoup
- [x] T017 [US1] Add content extraction logic using Docusaurus-specific selectors (<article>, <main>, .markdown)
- [x] T018 [US1] Extract page title from <h1> or <title> tags in extract_text_from_url()
- [x] T019 [US1] Extract breadcrumb hierarchy from .breadcrumbs class in extract_text_from_url()
- [x] T020 [US1] Add error handling for network failures (use retry helper) in get_all_urls() and extract_text_from_url()
- [x] T021 [US1] Add validation to ensure extracted content is non-empty in extract_text_from_url()
- [x] T022 [US1] Update main() function to orchestrate URL discovery and content extraction with progress logging

**Deliverables**:
- `get_all_urls()` function
- `extract_text_from_url()` function
- Integration in `main()` with logging

**Parallel Execution Example** (within US1):
```bash
# These can be developed in parallel:
# - T014-T015: URL discovery logic
# - T016-T021: Content extraction logic
# Then integrate both in T022
```

**Independent Test** (US1 complete):
```bash
# Run with a test Docusaurus site
export BASE_URL=https://docusaurus.io
uv run python main.py

# Verify output:
# - Logs show "Discovered N URLs"
# - Logs show "Processed N/N pages"
# - No extraction errors
```

**Acceptance Validation** (maps to spec.md scenarios):
1. ✅ All pages discovered from sitemap
2. ✅ Content structure preserved (check extracted text includes headings, code blocks)
3. ✅ Hierarchy captured (check section_hierarchy field is populated)

---

## Phase 4: User Story 2 - Semantic Embedding Generation (P2)

**Story Goal**: Generate high-quality embeddings for extracted documentation chunks with intelligent chunking and context preservation.

**Dependencies**: Requires US1 complete (extracted content)

**Independent Test Criteria** (from spec.md):
- Provide sample content → chunks created with appropriate size
- Generate embeddings → verify 1024-dimensional vectors
- Similar content produces similar embeddings (high cosine similarity)

**Tasks**:

- [x] T023 [P] [US2] Implement chunk_text() function in backend/main.py with character-based sliding window (1000 chars, 200 overlap)
- [x] T024 [P] [US2] Add code block detection logic in chunk_text() (detect ``` or <pre><code> patterns)
- [x] T025 [US2] Implement adaptive code block handling: keep small blocks (<500 chars) with prose, split large blocks
- [x] T026 [US2] Generate chunk_id (UUID) and chunk_index for each chunk in chunk_text()
- [x] T027 [US2] Detect content_type ("prose", "code", "mixed") for each chunk in chunk_text()
- [x] T028 [US2] Extract nearest heading for each chunk in chunk_text()
- [x] T029 [P] [US2] Implement embed_chunks() function in backend/main.py to call Cohere API with batch support
- [x] T030 [US2] Initialize Cohere client in main() function with API key from config
- [x] T031 [US2] Add batch processing logic in embed_chunks() (process 50 chunks per request)
- [x] T032 [US2] Implement rate limit handling with exponential backoff for Cohere 429 errors in embed_chunks()
- [x] T033 [US2] Add progress logging for chunking and embedding operations in main()
- [x] T034 [US2] Update main() to orchestrate chunking and embedding after content extraction

**Deliverables**:
- `chunk_text()` function with adaptive logic
- `embed_chunks()` function with batch processing
- Cohere client initialization
- Integration in `main()`

**Parallel Execution Example** (within US2):
```bash
# These can be developed in parallel:
# - T023-T028: Chunking logic
# - T029-T032: Embedding logic
# Then integrate both in T033-T034
```

**Independent Test** (US2 complete):
```bash
# Run with chunking and embedding enabled
uv run python main.py

# Verify output:
# - Logs show "Created N chunks"
# - Logs show "Embedded N/N chunks"
# - Each chunk has 1024-dimensional embedding
# - No Cohere API errors
```

**Acceptance Validation** (maps to spec.md scenarios):
1. ✅ Content chunked intelligently (check chunk sizes ~1000 chars)
2. ✅ Code blocks handled correctly (small blocks with prose, large blocks separate)
3. ✅ Embeddings generated (check embedding field exists, length = 1024)
4. ✅ Similar content has high similarity (manually test with similar chunks)

---

## Phase 5: User Story 3 - Vector Database Storage and Indexing (P3)

**Story Goal**: Store embeddings and metadata in Qdrant Cloud with efficient similarity search capability.

**Dependencies**: Requires US2 complete (embeddings generated)

**Independent Test Criteria** (from spec.md):
- Store embeddings with metadata → verify vectors persisted
- Similarity search returns ranked results
- Metadata correctly associated with each vector

**Tasks**:

- [x] T035 [P] [US3] Implement create_collection() function in backend/main.py to initialize Qdrant collection
- [x] T036 [US3] Configure collection with VectorParams (size=1024, distance=Cosine) in create_collection()
- [x] T037 [US3] Add idempotent check (skip if collection exists) in create_collection()
- [x] T038 [P] [US3] Implement save_chunks_to_qdrant() function in backend/main.py to upsert vectors in batches
- [x] T039 [US3] Transform ContentChunk to Qdrant PointStruct format (id, vector, payload) in save_chunks_to_qdrant()
- [x] T040 [US3] Add batch upsert logic (100 points per request) in save_chunks_to_qdrant()
- [x] T041 [US3] Add progress logging for Qdrant operations in save_chunks_to_qdrant()
- [x] T042 [P] [US3] Implement verify_ingestion() function in backend/main.py to perform sample similarity search
- [x] T043 [US3] Generate query embedding using Cohere (input_type="search_query") in verify_ingestion()
- [x] T044 [US3] Perform Qdrant similarity search (limit=5) in verify_ingestion()
- [x] T045 [US3] Log top result with score and metadata in verify_ingestion()
- [x] T046 [US3] Initialize Qdrant client in main() function with URL and API key from config
- [x] T047 [US3] Update main() to orchestrate collection creation, vector storage, and verification

**Deliverables**:
- `create_collection()` function
- `save_chunks_to_qdrant()` function
- `verify_ingestion()` function
- Qdrant client initialization
- Complete pipeline in `main()`

**Parallel Execution Example** (within US3):
```bash
# These can be developed in parallel:
# - T035-T037: Collection creation
# - T038-T041: Vector storage
# - T042-T045: Verification
# Then integrate all in T046-T047
```

**Independent Test** (US3 complete):
```bash
# Run complete pipeline
uv run python main.py

# Verify output:
# - Logs show "Collection 'rag_embedding' created" or "already exists"
# - Logs show "Saved N/N chunks to Qdrant"
# - Logs show "Verification query: ..." with top result
# - Top result has score > 0.5
```

**Acceptance Validation** (maps to spec.md scenarios):
1. ✅ Vectors persisted with metadata (check Qdrant dashboard shows N vectors)
2. ✅ Similarity search returns ranked results (verify_ingestion logs results)
3. ✅ Re-ingestion updates without duplication (run twice, vector count same)
4. ✅ Performance acceptable (search returns in <200ms per spec SC-005)

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Finalize documentation, error handling, and edge case coverage

**Tasks**:

- [x] T048 Add comprehensive docstrings to all functions in backend/main.py (follow contracts/schemas.md format)
- [x] T049 Add type annotations to all function signatures in backend/main.py
- [x] T050 Improve error messages for common failures (missing credentials, network errors, API errors)
- [x] T051 Add summary statistics logging at end of main() (total URLs, chunks, embeddings, storage time)
- [x] T052 Create backend/README.md with setup instructions (reference quickstart.md)
- [x] T053 Test end-to-end pipeline with a real Docusaurus site (e.g., https://docusaurus.io)
- [ ] T054 [OPTIONAL] Verify rate limit handling by testing with large documentation site (500+ pages)
- [x] T055 Document edge cases and limitations in backend/README.md (JS-rendered sites, rate limits, etc.)

**Deliverables**:
- Fully documented `backend/main.py`
- `backend/README.md` with setup guide
- Verified end-to-end functionality

**Independent Test** (Full Pipeline):
```bash
# Test with real Docusaurus site
export BASE_URL=https://docusaurus.io
export COHERE_API_KEY=your_key
export QDRANT_URL=your_url
export QDRANT_API_KEY=your_key
uv run python main.py

# Expected output:
# INFO: Starting RAG content ingestion...
# INFO: Discovered 150+ URLs
# INFO: Processed 150/150 pages
# INFO: Created 2000+ chunks
# INFO: Embedded 2000/2000 chunks
# INFO: Saved 2000 chunks to Qdrant
# INFO: Top result: "Introduction" (score: 0.92)
# INFO: Ingestion complete!
```

---

## Task Summary

**Total Tasks**: 55

**Breakdown by Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 5 tasks
- Phase 3 (US1 - Content Extraction): 9 tasks
- Phase 4 (US2 - Embedding Generation): 12 tasks
- Phase 5 (US3 - Vector Storage): 13 tasks
- Phase 6 (Polish): 8 tasks

**Breakdown by User Story**:
- US1 (P1): 9 tasks (T014-T022)
- US2 (P2): 12 tasks (T023-T034)
- US3 (P3): 13 tasks (T035-T047)
- Setup/Foundational: 13 tasks (T001-T013)
- Polish: 8 tasks (T048-T055)
- US4 (P4): 0 tasks (out of scope for MVP)

**Parallel Opportunities**: 15 tasks marked with [P]

**Suggested MVP Scope**: Phases 1-6 (excludes US4 - Incremental Updates)

---

## File Reference

All implementation happens in:
- **Primary**: `backend/main.py` (single file, ~400-500 lines)
- **Config**: `backend/.env`, `backend/pyproject.toml`
- **Docs**: `backend/README.md`

**No separate test files required** - verification happens through:
1. Manual execution with logging output
2. Similarity search verification (`verify_ingestion()`)
3. Qdrant dashboard inspection

---

## Execution Workflow

### Day 1: Setup + US1 (Content Extraction)
```bash
# Phase 1: Setup (T001-T008)
mkdir backend && cd backend
uv init && uv python pin 3.11
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv lxml
# Create .env files, update .gitignore

# Phase 2: Foundational (T009-T013)
# Implement load_config(), retry logic, logging

# Phase 3: US1 (T014-T022)
# Implement get_all_urls(), extract_text_from_url()
# Test: Extract from https://docusaurus.io
```

### Day 2: US2 (Embedding Generation)
```bash
# Phase 4: US2 (T023-T034)
# Implement chunk_text(), embed_chunks()
# Test: Verify chunks created and embeddings generated
```

### Day 3: US3 (Vector Storage) + Polish
```bash
# Phase 5: US3 (T035-T047)
# Implement create_collection(), save_chunks_to_qdrant(), verify_ingestion()
# Test: Complete pipeline with similarity search

# Phase 6: Polish (T048-T055)
# Add documentation, improve error handling
# Final E2E test with large site
```

---

## Next Steps

1. Execute tasks in sequential order (T001 → T055)
2. Mark tasks complete with `[x]` after verification
3. For parallel tasks within a story, develop simultaneously then integrate
4. Test each user story independently before proceeding to next
5. Skip US4 (Incremental Updates) for MVP - plan for future iteration

## Success Criteria

Pipeline is complete when:
- ✅ All US1-US3 tasks completed (US4 out of scope)
- ✅ End-to-end test passes with real Docusaurus site
- ✅ Similarity search returns relevant results (score > 0.7)
- ✅ All error handling tested (missing credentials, network failures)
- ✅ Documentation complete (README.md, docstrings)
- ✅ No unhandled exceptions in normal operation
