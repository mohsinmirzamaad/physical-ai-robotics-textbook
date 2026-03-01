# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2026-03-01
**Status**: Complete

## Overview

This document defines the data entities, relationships, and validation rules for the Physical AI textbook platform.

## Entity Relationship Diagram

```
User (1) ──────< (M) Session
  │
  │ (1)
  │
  ├──────< (1) UserPreferences
  │
  └──────< (M) ChatMessage

Chapter (1) ──────< (M) ContentChunk
  │
  └──────< (M) ChatMessage
```

## Entities

### 1. User

Represents a student or reader of the textbook.

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, AUTO | Unique user identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | User email address |
| password_hash | VARCHAR(255) | NOT NULL | Bcrypt hashed password |
| software_experience | ENUM | NOT NULL | 'beginner', 'intermediate', 'advanced' |
| hardware_experience | ENUM | NOT NULL | 'none', 'hobbyist', 'professional' |
| email_verified | BOOLEAN | DEFAULT false | Email verification status |
| created_at | TIMESTAMP | DEFAULT NOW() | Account creation timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Validation Rules:**
- Email must be valid format (RFC 5322)
- Password minimum length: 10 characters
- Password must contain: uppercase, lowercase, number, special character
- Software experience must be one of: beginner, intermediate, advanced
- Hardware experience must be one of: none, hobbyist, professional

**Indexes:**
- PRIMARY KEY on id
- UNIQUE INDEX on email
- INDEX on created_at (for analytics)

**Relationships:**
- One-to-Many with Session
- One-to-One with UserPreferences
- One-to-Many with ChatMessage

---

### 2. Session

Represents an authenticated user session.

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, AUTO | Unique session identifier |
| user_id | UUID | FOREIGN KEY, NOT NULL | Reference to User |
| token | VARCHAR(255) | UNIQUE, NOT NULL | Session token (JWT or random) |
| expires_at | TIMESTAMP | NOT NULL | Session expiration time |
| ip_address | VARCHAR(45) | NULL | Client IP address (IPv4/IPv6) |
| user_agent | TEXT | NULL | Browser user agent string |
| created_at | TIMESTAMP | DEFAULT NOW() | Session creation timestamp |

**Validation Rules:**
- Token must be cryptographically secure (32+ bytes)
- Expires_at must be future timestamp
- Session duration: 7 days from creation
- IP address must be valid IPv4 or IPv6 format

**Indexes:**
- PRIMARY KEY on id
- UNIQUE INDEX on token
- INDEX on user_id (for user session lookup)
- INDEX on expires_at (for cleanup queries)

**Relationships:**
- Many-to-One with User

**Lifecycle:**
- Created on successful login
- Deleted on logout or expiration
- Cleanup job runs daily to remove expired sessions

---

### 3. UserPreferences

Stores user-specific preferences for personalization and language.

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | PRIMARY KEY, FOREIGN KEY | Reference to User |
| personalization_enabled | BOOLEAN | DEFAULT false | Enable content personalization |
| preferred_language | VARCHAR(10) | DEFAULT 'en' | Language code (ISO 639-1) |
| last_chapter_visited | VARCHAR(100) | NULL | Last chapter slug for resume |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last preference update |

**Validation Rules:**
- Preferred language must be one of: 'en', 'ur'
- Last chapter visited must match existing chapter slug

**Indexes:**
- PRIMARY KEY on user_id
- INDEX on updated_at

**Relationships:**
- One-to-One with User

---

### 4. Chapter

Represents a chapter or section of the textbook (metadata only, content stored in files).

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, AUTO | Unique chapter identifier |
| slug | VARCHAR(100) | UNIQUE, NOT NULL | URL-friendly identifier |
| title | VARCHAR(255) | NOT NULL | Chapter title |
| module | ENUM | NOT NULL | 'ros2', 'digital-twin', 'isaac', 'vla' |
| week_number | INTEGER | NOT NULL | Week number (1-13) |
| order_index | INTEGER | NOT NULL | Display order within module |
| file_path | VARCHAR(255) | NOT NULL | Path to markdown file |
| estimated_reading_time | INTEGER | NULL | Minutes to read |
| created_at | TIMESTAMP | DEFAULT NOW() | Chapter creation timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last content update |

**Validation Rules:**
- Slug must be lowercase, alphanumeric with hyphens
- Module must be one of: ros2, digital-twin, isaac, vla
- Week number must be between 1 and 13
- Order index must be unique within module
- File path must exist in docs/ directory

**Indexes:**
- PRIMARY KEY on id
- UNIQUE INDEX on slug
- INDEX on module, order_index (for navigation)
- INDEX on week_number

**Relationships:**
- One-to-Many with ContentChunk
- One-to-Many with ChatMessage

---

### 5. ContentChunk

Represents a chunk of textbook content stored in the vector database for RAG retrieval.

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, AUTO | Unique chunk identifier |
| chapter_id | UUID | FOREIGN KEY, NOT NULL | Reference to Chapter |
| content | TEXT | NOT NULL | Actual text content |
| embedding | VECTOR(1536) | NOT NULL | OpenAI embedding vector |
| chunk_index | INTEGER | NOT NULL | Position within chapter |
| section_heading | VARCHAR(255) | NULL | Parent section title |
| token_count | INTEGER | NOT NULL | Number of tokens in chunk |
| created_at | TIMESTAMP | DEFAULT NOW() | Chunk creation timestamp |

**Validation Rules:**
- Content length: 200-1000 tokens
- Chunk index must be sequential within chapter
- Token count must match actual content
- Embedding dimension must be 1536

**Indexes:**
- PRIMARY KEY on id
- INDEX on chapter_id, chunk_index
- VECTOR INDEX on embedding (HNSW in Qdrant)

**Relationships:**
- Many-to-One with Chapter

**Storage:**
- Metadata stored in Postgres
- Vectors stored in Qdrant Cloud
- Synchronized via chunk_id

---

### 6. ChatMessage

Represents a conversation between user and chatbot.

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, AUTO | Unique message identifier |
| user_id | UUID | FOREIGN KEY, NULL | Reference to User (null for anonymous) |
| chapter_id | UUID | FOREIGN KEY, NULL | Chapter context |
| user_query | TEXT | NOT NULL | User's question |
| selected_text | TEXT | NULL | Text selected by user |
| bot_response | TEXT | NOT NULL | Chatbot's answer |
| response_time_ms | INTEGER | NULL | Response generation time |
| retrieved_chunks | INTEGER | NULL | Number of chunks retrieved |
| created_at | TIMESTAMP | DEFAULT NOW() | Message timestamp |

**Validation Rules:**
- User query length: 10-1000 characters
- Selected text length: max 2000 characters
- Response time must be positive integer
- Retrieved chunks: 0-10

**Indexes:**
- PRIMARY KEY on id
- INDEX on user_id, created_at (for history)
- INDEX on chapter_id (for analytics)
- INDEX on created_at (for cleanup)

**Relationships:**
- Many-to-One with User (optional)
- Many-to-One with Chapter (optional)

**Lifecycle:**
- Created on each chatbot interaction
- Retained for 30 days for analytics
- Cleanup job removes old messages

---

## State Transitions

### User Account States

```
[New] ──register──> [Unverified] ──verify_email──> [Active]
                                                      │
                                                      │ login
                                                      ↓
                                                   [Authenticated]
                                                      │
                                                      │ logout/expire
                                                      ↓
                                                   [Active]
```

### Session States

```
[Created] ──use──> [Active] ──expire/logout──> [Expired/Deleted]
```

### Content Chunk States

```
[New] ──embed──> [Indexed] ──update──> [Reindexed]
```

## Data Validation Summary

### User Input Validation

**Email:**
- Format: RFC 5322 compliant
- Max length: 255 characters
- Uniqueness check before registration

**Password:**
- Min length: 10 characters
- Must contain: uppercase, lowercase, number, special character
- Hashed with bcrypt (cost factor: 12)

**Experience Levels:**
- Software: beginner | intermediate | advanced
- Hardware: none | hobbyist | professional

**Chat Query:**
- Min length: 10 characters
- Max length: 1000 characters
- Sanitize HTML/script tags

**Selected Text:**
- Max length: 2000 characters
- Must be from current chapter

### System Constraints

**Session Management:**
- Max sessions per user: 5 (oldest auto-deleted)
- Session duration: 7 days
- Token length: 64 characters (hex)

**Content Chunking:**
- Chunk size: 500-800 tokens
- Overlap: 100-150 tokens
- Max chunks per chapter: 100

**Vector Search:**
- Top K results: 5
- Score threshold: 0.7
- Max query time: 500ms

## Database Schema (SQL)

```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_experience VARCHAR(20) NOT NULL CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(20) NOT NULL CHECK (hardware_experience IN ('none', 'hobbyist', 'professional')),
    email_verified BOOLEAN DEFAULT false,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);

-- Sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    ip_address VARCHAR(45),
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE UNIQUE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);

-- User preferences table
CREATE TABLE user_preferences (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    personalization_enabled BOOLEAN DEFAULT false,
    preferred_language VARCHAR(10) DEFAULT 'en' CHECK (preferred_language IN ('en', 'ur')),
    last_chapter_visited VARCHAR(100),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_user_preferences_updated_at ON user_preferences(updated_at);

-- Chapters table
CREATE TABLE chapters (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    slug VARCHAR(100) UNIQUE NOT NULL,
    title VARCHAR(255) NOT NULL,
    module VARCHAR(20) NOT NULL CHECK (module IN ('ros2', 'digital-twin', 'isaac', 'vla')),
    week_number INTEGER NOT NULL CHECK (week_number BETWEEN 1 AND 13),
    order_index INTEGER NOT NULL,
    file_path VARCHAR(255) NOT NULL,
    estimated_reading_time INTEGER,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(module, order_index)
);

CREATE UNIQUE INDEX idx_chapters_slug ON chapters(slug);
CREATE INDEX idx_chapters_module_order ON chapters(module, order_index);
CREATE INDEX idx_chapters_week ON chapters(week_number);

-- Content chunks table (metadata only, vectors in Qdrant)
CREATE TABLE content_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
    content TEXT NOT NULL,
    chunk_index INTEGER NOT NULL,
    section_heading VARCHAR(255),
    token_count INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(chapter_id, chunk_index)
);

CREATE INDEX idx_content_chunks_chapter ON content_chunks(chapter_id, chunk_index);

-- Chat messages table
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    chapter_id UUID REFERENCES chapters(id) ON DELETE SET NULL,
    user_query TEXT NOT NULL,
    selected_text TEXT,
    bot_response TEXT NOT NULL,
    response_time_ms INTEGER,
    retrieved_chunks INTEGER,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chat_messages_user ON chat_messages(user_id, created_at);
CREATE INDEX idx_chat_messages_chapter ON chat_messages(chapter_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);

-- Cleanup function for expired sessions
CREATE OR REPLACE FUNCTION cleanup_expired_sessions()
RETURNS void AS $$
BEGIN
    DELETE FROM sessions WHERE expires_at < NOW();
END;
$$ LANGUAGE plpgsql;

-- Cleanup function for old chat messages (30 days retention)
CREATE OR REPLACE FUNCTION cleanup_old_chat_messages()
RETURNS void AS $$
BEGIN
    DELETE FROM chat_messages WHERE created_at < NOW() - INTERVAL '30 days';
END;
$$ LANGUAGE plpgsql;
```

## Qdrant Collection Schema

```python
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

collection_config = {
    "collection_name": "textbook_embeddings",
    "vectors_config": VectorParams(
        size=1536,
        distance=Distance.COSINE
    ),
    "payload_schema": {
        "chunk_id": PayloadSchemaType.KEYWORD,
        "chapter_id": PayloadSchemaType.KEYWORD,
        "chapter_slug": PayloadSchemaType.KEYWORD,
        "chapter_title": PayloadSchemaType.TEXT,
        "module": PayloadSchemaType.KEYWORD,
        "week_number": PayloadSchemaType.INTEGER,
        "section_heading": PayloadSchemaType.TEXT,
        "content": PayloadSchemaType.TEXT,
        "token_count": PayloadSchemaType.INTEGER
    },
    "hnsw_config": {
        "m": 16,
        "ef_construct": 100
    }
}
```

## Data Migration Strategy

### Initial Setup

1. **Create Postgres schema** using SQL above
2. **Create Qdrant collection** with configuration
3. **Seed chapters table** from content directory structure
4. **Generate content chunks** from markdown files
5. **Embed and index chunks** in Qdrant

### Content Updates

1. **Detect changed files** via git diff or file hash
2. **Delete old chunks** for changed chapters
3. **Re-chunk and re-embed** updated content
4. **Update Qdrant collection** with new vectors
5. **Update chapter metadata** (updated_at timestamp)

## Data Retention Policy

| Entity | Retention Period | Cleanup Method |
|--------|------------------|----------------|
| Users | Indefinite | Manual deletion on request |
| Sessions | 7 days | Automated daily cleanup |
| UserPreferences | Indefinite | Deleted with user |
| Chapters | Indefinite | Manual content management |
| ContentChunks | Indefinite | Updated with content |
| ChatMessages | 30 days | Automated weekly cleanup |

## Privacy & Security

**PII (Personally Identifiable Information):**
- Email addresses (encrypted at rest)
- IP addresses (optional, for security only)

**Non-PII:**
- Experience levels (aggregated for analytics)
- Chat queries (anonymized after 30 days)
- Session tokens (cryptographically secure)

**Data Protection:**
- Passwords hashed with bcrypt (never stored plaintext)
- Session tokens HTTP-only cookies
- Database connections encrypted (SSL/TLS)
- API keys stored in environment variables
- Regular backups (Neon automatic backups)

**GDPR Compliance:**
- Right to access: User can export their data
- Right to deletion: User can delete account and all data
- Right to rectification: User can update preferences
- Data minimization: Only collect necessary data

## Conclusion

Data model supports all feature requirements with proper validation, relationships, and security. Ready for API contract definition.
