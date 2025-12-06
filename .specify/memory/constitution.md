<!-- Sync Impact Report:
Version change: 0.0.0 -> 0.1.0
Modified principles:
  - PRINCIPLE_1_NAME -> Spec-driven writing
  - PRINCIPLE_2_NAME -> Technical accuracy
  - PRINCIPLE_3_NAME -> Clarity
  - PRINCIPLE_4_NAME -> Reproducibility
  - PRINCIPLE_5_NAME -> Tool-first workflow
Added sections: Standards, Constraints
Removed sections:
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: none
-->
# AI/Spec-Driven Book Creation using Docusaurus + GitHub Pages + Spec-Kit Plus + Claude Code Constitution

## Core Principles

### Spec-driven writing
All decisions and changes must be documented; no undocumented decisions.

### Technical accuracy
All code and configuration must be tested and verified for technical accuracy.

### Clarity
Content must be beginner–intermediate friendly, ensuring easy understanding for the target audience.

### Reproducibility
Every step and example provided must be fully reproducible by readers without issues.

### Tool-first workflow
The entire book creation and development process must utilize Spec-Kit Plus + Claude Code throughout.

## Standards

- All commands and code examples must be verified locally before inclusion.
- Consistent versions of development tools (Node, Docusaurus, Git) must be maintained.
- All code blocks provided in the book must be copy-paste ready and executable.
- Proper folder structure and naming conventions must be followed.
- No hallucinated features or content will be included.
- Must include relevant examples, exercises, and diagrams to enhance learning.

## Constraints

- The project must be built with Docusaurus and successfully pass `npm run build`.
- The book must be deployed on GitHub Pages.
- The book must contain a minimum of 8 chapters, with each chapter having at least 3 sections.
- The total length of the book must be between 15,000 and 25,000 words.
- The readability level of the content must be between grade 8 and 12.
- All content must be original, ensuring zero plagiarism.

## Governance

- All specifications must be completed and approved before implementation.
- Code implemented must work exactly as written and described in the book.
- The Docusaurus build process must complete with 0 errors.
- Deployment to GitHub Pages must be successful.
- Chapters are clear, consistent, and reproducible.
- The Spec-Kit Plus history must be clean and organized.

**Version**: 0.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
