# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2026-03-01 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Build an AI-native educational textbook platform for teaching Physical AI & Humanoid Robotics. The platform consists of a Docusaurus-based static site with 13 weeks of content across 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA), an embedded RAG chatbot powered by OpenAI and Qdrant for interactive learning, user authentication with Better-Auth for personalized experiences, and content personalization/translation features. The system targets 100+ concurrent users with <3 second chatbot response times and 90%+ answer accuracy.

## Technical Context

**Language/Version**:
- Frontend: JavaScript/TypeScript with Node.js 18+, React 18.2+
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: Docusaurus 3.6.3, Better-Auth 1.3.10, React 18.2
- Backend: FastAPI 0.109+, OpenAI Python SDK, Qdrant Client, asyncpg

**Storage**:
- Relational: Neon Serverless Postgres (users, sessions, preferences, chapters metadata)
- Vector: Qdrant Cloud (content embeddings for RAG)
- Cache: Redis (optional, for embedding/query caching)

**Testing**:
- Frontend: Jest, React Testing Library
- Backend: pytest, pytest-asyncio
- Integration: Playwright for E2E

**Target Platform**:
- Deployment: Vercel (static site + serverless functions)
- Browser: Modern browsers (Chrome, Firefox, Safari, Edge)
- Mobile: Responsive web (no native app)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page load: <2 seconds (first contentful paint)
- Chatbot response: <3 seconds average
- API p95 latency: <500ms
- Concurrent users: 100+ without degradation

**Constraints**:
- Free tier limits: Neon (512 MB), Qdrant (1 GB), Vercel (bandwidth)
- OpenAI API rate limits and costs
- Chatbot accuracy: 90%+ on test question set

**Scale/Scope**:
- Content: 13 weeks, 4 modules, ~50 chapters
- Users: 100-500 expected during hackathon evaluation
- Embeddings: 5,000-10,000 content chunks
- Chat queries: ~1,000/day estimated

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. AI-Native Content First ✅ PASS

**Requirement**: Content structured for both human readers and AI agents (RAG chatbot)

**Compliance**:
- Markdown-based documentation with semantic structure (Docusaurus)
- Content chunked and embedded for RAG retrieval (Qdrant)
- Clear, parseable sections enable context-aware AI responses
- Code snippets formatted for easy extraction and explanation

**Evidence**: research.md documents chunking strategy (500-800 tokens, semantic splitting), data-model.md defines ContentChunk entity with embeddings

### II. Accessibility & Personalization ✅ PASS

**Requirement**: Support for multiple languages, personalization based on user background

**Compliance**:
- Multi-language support: English and Urdu translation
- Personalization based on software/hardware experience levels
- Progressive disclosure via three content variants (beginner/intermediate/advanced)
- Clear learning paths through modular structure

**Evidence**: API contracts include `/content/personalize` and `/content/translate` endpoints, data-model.md includes UserPreferences entity

### III. Modular Content Architecture ✅ PASS

**Requirement**: Self-contained, reusable modules with consistent structure

**Compliance**:
- Content organized in 4 modules with clear prerequisites
- Consistent structure: Learning Outcomes → Content → Assessments
- Each chapter stands alone (Chapter entity with metadata)
- Cross-references enhance but don't create dependencies

**Evidence**: Project structure shows modular organization (module-1-ros2/, module-2-digital-twin/, etc.), quickstart.md documents content template

### IV. Practical, Simulation-First Learning ✅ PASS

**Requirement**: Hands-on, executable examples with simulation environments

**Compliance**:
- Content covers simulation tools (Gazebo, Isaac Sim, Unity)
- Code examples included in markdown with syntax highlighting
- Progressive complexity from simple to advanced
- Hardware requirements documented in content

**Evidence**: Course outline includes simulation modules (weeks 6-10), Docusaurus config includes syntax highlighting for Python, C++, YAML

### V. Quality & Accuracy (NON-NEGOTIABLE) ✅ PASS

**Requirement**: Code examples tested, technical specs current, citations included

**Compliance**:
- Testing framework in place (pytest, Jest)
- Version-specific dependencies documented (Docusaurus 3.6.3, Python 3.11+)
- Chatbot accuracy target: 90%+ on test question set
- Regular reviews via git version control

**Evidence**: quickstart.md includes testing commands, research.md documents current versions with rationale

### VI. Open & Collaborative ✅ PASS

**Requirement**: Version controlled, spec-driven development, documented decisions

**Compliance**:
- Git version control with clear change history
- Spec-driven development using Spec-Kit Plus
- ADRs for significant decisions (to be created during implementation)
- PHRs for prompt history tracking

**Evidence**: This plan follows spec-driven workflow, PHRs created for constitution and specification phases

### Post-Design Re-check: ✅ ALL GATES PASS

All constitution principles satisfied. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - technology decisions
├── data-model.md        # Phase 1 output - entities and schema
├── quickstart.md        # Phase 1 output - setup guide
├── contracts/           # Phase 1 output - API contracts
│   └── api.yaml         # OpenAPI specification
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist
└── spec.md              # Feature specification
```

### Source Code (repository root)

```text
physical-ai-textbook/
├── docs/                          # Docusaurus content (Markdown)
│   ├── intro.md
│   ├── module-1-ros2/
│   │   ├── week-1-2-physical-ai-intro/
│   │   │   ├── foundations.md
│   │   │   ├── embodied-intelligence.md
│   │   │   └── sensor-systems.md
│   │   └── week-3-5-ros2-fundamentals/
│   │       ├── ros2-architecture.md
│   │       ├── nodes-topics-services.md
│   │       └── launch-files.md
│   ├── module-2-digital-twin/
│   │   └── week-6-7-gazebo-unity/
│   ├── module-3-nvidia-isaac/
│   │   └── week-8-10-isaac-platform/
│   └── module-4-vla/
│       ├── week-11-12-humanoid-dev/
│       └── week-13-conversational-robotics/
│
├── src/                           # Frontend React components
│   ├── components/
│   │   ├── ChatbotWidget.tsx     # Embedded chatbot UI
│   │   ├── SignupForm.tsx        # User registration
│   │   ├── LoginForm.tsx         # User login
│   │   ├── ForgotPassword.tsx    # Password reset request
│   │   ├── ResetPassword.tsx     # Password reset confirm
│   │   └── ProtectedChapter.tsx  # Auth wrapper for chapters
│   ├── lib/
│   │   ├── auth.ts               # Better-Auth server config
│   │   └── auth-client.ts        # Better-Auth React hooks
│   ├── css/
│   │   └── custom.css            # Custom styling
│   └── theme/
│       └── Root.tsx              # SessionProvider wrapper
│
├── api/                          # FastAPI backend
│   ├── main.py                   # FastAPI app entry point
│   ├── routes/
│   │   ├── auth.py               # Authentication endpoints
│   │   ├── chat.py               # Chatbot endpoints
│   │   └── content.py            # Personalization/translation
│   ├── services/
│   │   ├── embeddings.py         # OpenAI embedding generation
│   │   ├── qdrant_client.py      # Vector DB operations
│   │   ├── openai_client.py      # LLM chat operations
│   │   ├── personalization.py    # Content adaptation
│   │   └── translation.py        # Urdu translation
│   ├── models/
│   │   └── schemas.py            # Pydantic models
│   └── database/
│       ├── connection.py         # Neon Postgres connection
│       └── queries.py            # Database operations
│
├── scripts/
│   ├── embed-content.py          # Generate embeddings from docs/
│   ├── seed-database.py          # Seed chapters table
│   └── cleanup-sessions.py       # Maintenance script
│
├── tests/
│   ├── frontend/
│   │   ├── components/           # Component tests
│   │   └── integration/          # E2E tests
│   └── backend/
│       ├── unit/                 # Unit tests
│       └── integration/          # API integration tests
│
├── static/                       # Static assets
│   ├── img/
│   └── diagrams/
│
├── docusaurus.config.js          # Docusaurus configuration
├── sidebars.js                   # Sidebar navigation
├── package.json                  # Node dependencies
├── requirements.txt              # Python dependencies
├── vercel.json                   # Vercel deployment config
├── .env.example                  # Environment variables template
└── README.md                     # Project documentation
```

**Structure Decision**: Web application structure selected. Frontend uses Docusaurus for static site generation with React components for interactive features (auth, chatbot). Backend uses FastAPI for serverless API functions deployed alongside the static site on Vercel. This structure enables:
- Clear separation of concerns (content vs. logic)
- Independent scaling (static CDN vs. serverless functions)
- Unified deployment (single Vercel project)
- Shared environment variables and authentication

## Complexity Tracking

> **No violations - this section intentionally left empty**

All constitution principles are satisfied without requiring complexity justifications. The architecture follows best practices for educational content platforms with appropriate separation of concerns.
