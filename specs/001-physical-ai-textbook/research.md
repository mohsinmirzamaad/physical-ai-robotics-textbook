# Research: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2026-03-01
**Status**: Complete

## Overview

This document consolidates research findings for implementing an AI-native educational textbook with embedded RAG chatbot, authentication, personalization, and translation features.

## 1. Static Site Generator: Docusaurus

### Decision: Docusaurus 3.6.3

**Rationale:**
- Purpose-built for documentation and educational content
- Excellent MDX support for interactive components
- Built-in search, navigation, and mobile responsiveness
- Strong plugin ecosystem for extensibility
- React-based, enabling seamless integration with Better-Auth and chatbot widget

**Key Features:**
- Automatic sidebar generation from directory structure
- Syntax highlighting for 100+ languages (Python, C++, YAML, Bash)
- Tabs component for multi-language code examples
- Admonitions for callouts (tips, warnings, hardware requirements)
- Generated index pages for module overviews

**Content Organization:**
```
docs/
├── intro.md
├── module-1-ros2/
│   ├── week-1-2-physical-ai-intro/
│   ├── week-3-5-ros2-fundamentals/
├── module-2-digital-twin/
│   ├── week-6-7-gazebo-unity/
├── module-3-nvidia-isaac/
│   ├── week-8-10-isaac-platform/
└── module-4-vla/
    ├── week-11-12-humanoid-dev/
    └── week-13-conversational-robotics/
```

**Alternatives Considered:**
- **VitePress**: Lighter but less feature-rich for educational content
- **Nextra**: Good but less mature ecosystem
- **GitBook**: Commercial focus, limited customization

## 2. Deployment Platform: Vercel

### Decision: Vercel (over GitHub Pages)

**Rationale:**
- **Serverless Functions**: Critical for hosting FastAPI chatbot backend alongside static site
- **Preview Deployments**: Every PR gets unique URL for testing
- **Edge Network**: Global CDN for faster load times
- **Environment Variables**: Secure storage for API keys (OpenAI, Qdrant, Neon)
- **Zero Configuration**: Auto-detects Docusaurus
- **Build Performance**: Faster builds with intelligent caching

**GitHub Pages Comparison:**
- Pros: Free, simple GitHub integration
- Cons: No serverless functions (requires separate chatbot hosting), slower builds, no preview deployments

**Deployment Configuration:**
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "rewrites": [
    {"source": "/api/:path*", "destination": "/api/:path*"}
  ]
}
```

## 3. Authentication: Better-Auth

### Decision: Better-Auth v1.3.10 with Neon Postgres

**Rationale:**
- Framework-agnostic, works seamlessly with React/Docusaurus
- Native Neon Postgres support via standard PostgreSQL adapter
- Built-in security: bcrypt hashing, HTTP-only cookies, CSRF protection
- Custom fields support for background questionnaire
- Automatic schema management via migrations
- TypeScript-first with complete type inference

**Custom User Schema:**
```typescript
{
  email: string,
  password: string (hashed),
  softwareExperience: 'beginner' | 'intermediate' | 'advanced',
  hardwareExperience: 'none' | 'hobbyist' | 'professional',
  createdAt: timestamp,
  updatedAt: timestamp
}
```

**Session Management:**
- HTTP-only cookies with SameSite=Lax
- Automatic session persistence across page navigation
- No client-side storage needed
- Wrap Docusaurus with SessionProvider in `src/theme/Root.tsx`

**Password Reset Flow:**
- Token-based email verification (1 hour expiration)
- Two-component pattern: request form + reset form
- Requires email service (SendGrid, Resend, or similar)

**Alternatives Considered:**
- **NextAuth.js**: Excellent but Next.js-specific
- **Clerk**: Commercial, overkill for this use case
- **Auth0**: Enterprise focus, complex setup

## 4. RAG Chatbot Architecture

### Decision: OpenAI Agents SDK + Qdrant + FastAPI

**Component Breakdown:**

#### A. Embedding & Chunking Strategy

**Chunking Approach:**
- **Chunk Size**: 500-800 tokens (2-3 paragraphs)
- **Overlap**: 100-150 tokens (20% overlap) for context preservation
- **Semantic Splitting**: Respect markdown structure (headers, code blocks, lists)
- **Metadata**: Store chapter title, section heading, module, week number

**Embedding Model:**
- **Primary**: OpenAI text-embedding-3-small (1536 dimensions)
- **Cost/Performance**: Best balance for educational content
- **Alternative**: text-embedding-3-large (3072 dimensions) for higher accuracy but 2x cost

#### B. LLM Integration: OpenAI Agents SDK

**Rationale:**
- Built-in retrieval with native file search
- Streaming support for real-time responses
- Automatic thread management for conversation history
- Function calling for custom tools
- Production-ready error handling and rate limiting

**Model Choice:**
- **Primary**: GPT-4o (faster, cheaper than GPT-4 Turbo)
- **Fallback**: GPT-3.5-turbo for cost optimization

**Alternatives Considered:**
- **ChatKit SDK**: Lighter but requires manual RAG implementation
- **LangChain**: More complex, unnecessary overhead for this use case

#### C. Vector Database: Qdrant Cloud

**Configuration:**
- **Collection**: textbook_embeddings
- **Vector Size**: 1536 (text-embedding-3-small)
- **Distance Metric**: Cosine similarity
- **HNSW Parameters**: m=16, ef_construct=100 (balanced speed/accuracy)

**Indexing Strategy:**
- Bulk upload with indexing disabled initially
- Payload indexing on chapter, section, module fields
- Score threshold: 0.7 (only return relevant results)

**Free Tier Limits:**
- 1GB storage (~650,000 vectors at 1536 dimensions)
- Sufficient for textbook content (estimated 5,000-10,000 chunks)

#### D. Backend: FastAPI with Server-Sent Events (SSE)

**Rationale for SSE over WebSocket:**
- Simpler implementation (one-way streaming)
- Auto-reconnection built into browser
- HTTP/2 compatible
- Lower overhead (no handshake protocol)
- Perfect for streaming LLM responses

**API Endpoints:**
- `POST /api/chat/stream` - Streaming responses (SSE)
- `POST /api/chat` - Non-streaming responses
- `POST /api/embed` - Embed new content
- `GET /api/health` - Health check

**Performance Optimizations:**

1. **Multi-layer Caching:**
   - Layer 1: Semantic cache for similar queries (GPTCache)
   - Layer 2: Embedding cache (Redis, 1 hour TTL)
   - Layer 3: Qdrant query cache (5 minutes)

2. **Parallel Processing:**
   - Run embedding and metadata lookup concurrently
   - Connection pooling for Qdrant and OpenAI
   - Batch embeddings when possible

3. **Response Time Target: <3 seconds**
   - Embedding: ~200ms (cached: ~10ms)
   - Qdrant search: ~100ms
   - LLM streaming: ~2s (starts immediately)
   - Total: ~2.3s average

#### E. Selected-Text Query Handling

**Frontend:**
```typescript
const selectedText = window.getSelection().toString();
fetch('/api/chat/stream', {
    body: JSON.stringify({
        query: userQuestion,
        selectedText: selectedText,
        context: {chapter: currentChapter}
    })
});
```

**Backend Processing:**
- Prioritize selected text in prompt context
- Boost relevance by combining selected text + query in embedding
- Still perform vector search for supplementary information

## 5. Content Personalization

### Decision: Server-Side Dynamic Content Generation

**Approach:**
- Store three content variants per chapter: beginner, intermediate, advanced
- Generate variants using GPT-4o during content creation phase
- Serve appropriate variant based on user's background profile
- Cache personalized content per user session

**Personalization Logic:**
- **Beginner**: More explanatory text, commented code, prerequisite concepts
- **Intermediate**: Standard content (default)
- **Advanced**: Condensed basics, expanded advanced topics, less verbose code

**Implementation:**
- API endpoint: `GET /api/content/:chapter?level=beginner`
- Fallback to default (intermediate) if personalization fails
- "Reset to Default" button clears personalization preference

## 6. Translation: Urdu Support

### Decision: OpenAI GPT-4o for Translation

**Rationale:**
- Better context awareness than Google Translate for technical content
- Preserves technical terms in English
- Handles markdown formatting correctly
- Can be instructed to maintain code blocks untranslated

**Translation Strategy:**
- Translate on-demand (not pre-generated)
- Cache translations per chapter (Redis, 24 hour TTL)
- Preserve: code blocks, technical terms (ROS 2, NVIDIA Isaac, etc.)
- Translate: explanatory text, learning outcomes, assessments

**API Endpoint:**
- `GET /api/translate/:chapter?lang=ur`
- Toggle button at chapter header
- Language preference persists across chapters (session storage)

**Alternatives Considered:**
- **Google Translate API**: Cheaper but lower quality for technical content
- **Pre-generated translations**: Storage overhead, maintenance burden

## 7. Database: Neon Serverless Postgres

### Decision: Neon Postgres Free Tier

**Schema Design:**

**Users Table:**
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_experience VARCHAR(20) CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(20) CHECK (hardware_experience IN ('none', 'hobbyist', 'professional')),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
```

**Sessions Table:**
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);
```

**User Preferences Table:**
```sql
CREATE TABLE user_preferences (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    personalization_enabled BOOLEAN DEFAULT false,
    preferred_language VARCHAR(10) DEFAULT 'en',
    updated_at TIMESTAMP DEFAULT NOW()
);
```

**Free Tier Limits:**
- 512 MB storage
- 1 GB data transfer/month
- Sufficient for hackathon and initial users

## 8. Performance & Optimization

### Target Metrics

- **Page Load**: <2 seconds (first contentful paint)
- **Chatbot Response**: <3 seconds (average)
- **Concurrent Users**: 100+ without degradation
- **Chatbot Accuracy**: 90%+ on test question set

### Optimization Strategies

1. **Static Site:**
   - Code splitting and lazy loading
   - Image optimization (WebP, lazy load)
   - Bundle analysis and tree shaking
   - CDN delivery via Vercel Edge Network

2. **Chatbot:**
   - Multi-layer caching (semantic, embedding, query)
   - Parallel processing (embedding + metadata)
   - Connection pooling
   - Streaming responses (SSE)

3. **Database:**
   - Connection pooling (max 10 connections)
   - Indexed queries on email, user_id
   - Session cleanup cron job

4. **Monitoring:**
   - Prometheus metrics for latency, errors
   - Logging with structured JSON
   - Health check endpoints

## 9. Security Considerations

### Authentication
- Bcrypt password hashing (cost factor: 12)
- HTTP-only cookies with SameSite=Lax
- CSRF protection via Better-Auth
- Rate limiting on login attempts (5 per minute)
- Email verification for password reset

### API Security
- CORS configuration (whitelist Vercel domain)
- API key rotation for OpenAI, Qdrant
- Environment variables for secrets (never commit)
- Input validation and sanitization
- SQL injection prevention (parameterized queries)

### Data Privacy
- Minimal data collection (email, experience levels only)
- No PII in chat logs
- Session expiration (7 days)
- GDPR-compliant data deletion on request

## 10. Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| Static Site | Docusaurus | 3.6.3 | Purpose-built for docs, excellent MDX support |
| Deployment | Vercel | Latest | Serverless functions, preview deployments |
| Backend | FastAPI | 0.109+ | High performance, async support, SSE streaming |
| Language | Python | 3.11+ | FastAPI requirement, excellent AI library support |
| Authentication | Better-Auth | 1.3.10 | Framework-agnostic, custom fields, Neon support |
| Database | Neon Postgres | Latest | Serverless, free tier, Better-Auth compatible |
| Vector DB | Qdrant Cloud | Latest | Free tier, excellent performance, easy integration |
| LLM | OpenAI GPT-4o | Latest | Fast, cost-effective, streaming support |
| Embeddings | text-embedding-3-small | Latest | Best cost/performance for educational content |
| Cache | Redis | 7.0+ | Embedding and query caching |
| Frontend | React | 18.2+ | Docusaurus requirement |
| Node.js | Node.js | 18+ | Docusaurus requirement |

## 11. Development Timeline Estimate

**Week 1: Foundation**
- Docusaurus setup and configuration
- Content structure and sidebar organization
- Basic styling and theme customization

**Week 2: Content Creation**
- Write textbook content (13 weeks, 4 modules)
- Code examples and diagrams
- Learning outcomes and assessments

**Week 3: Authentication**
- Better-Auth integration
- Neon Postgres setup and schema
- Signup/login/reset flows
- Session management

**Week 4: RAG Chatbot**
- FastAPI backend setup
- Content chunking and embedding
- Qdrant integration
- OpenAI Agents SDK integration
- Chatbot widget component

**Week 5: Advanced Features**
- Content personalization
- Urdu translation
- Performance optimization
- Testing and bug fixes

**Week 6: Deployment & Polish**
- Vercel deployment
- Environment configuration
- Demo video creation
- Final testing

## 12. Open Questions Resolved

All technical unknowns from the specification have been resolved through research:

✅ Static site generator: Docusaurus 3.6.3
✅ Deployment platform: Vercel
✅ Authentication library: Better-Auth 1.3.10
✅ Database: Neon Serverless Postgres
✅ Vector database: Qdrant Cloud
✅ LLM integration: OpenAI Agents SDK with GPT-4o
✅ Embedding model: text-embedding-3-small
✅ Backend framework: FastAPI with SSE
✅ Translation approach: OpenAI GPT-4o
✅ Personalization strategy: Server-side dynamic content
✅ Caching layer: Redis

## 13. Risk Mitigation

**Risk: OpenAI API costs exceed budget**
- Mitigation: Multi-layer caching, use GPT-3.5-turbo fallback, rate limiting

**Risk: Free tier limits exceeded**
- Mitigation: Monitor usage, implement cleanup strategies, upgrade plan ready

**Risk: Translation quality poor**
- Mitigation: Manual review of critical sections, disclaimer about automated translation

**Risk: Chatbot accuracy below 90%**
- Mitigation: Comprehensive test question set, confidence scoring, iterative prompt tuning

**Risk: Content creation time insufficient**
- Mitigation: Prioritize core modules (ROS 2, Isaac), leverage existing resources, focus on depth

## Conclusion

All technical decisions have been made with clear rationale. The architecture is production-ready, scalable within free tier limits, and optimized for the hackathon requirements. Ready to proceed to Phase 1 (Design & Contracts).
