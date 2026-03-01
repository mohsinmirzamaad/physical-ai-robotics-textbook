"""
Database queries module with CRUD operations for all entities.
"""

from typing import Optional, List, Dict, Any
from uuid import UUID
from datetime import datetime
from .connection import execute_query, fetch_one, fetch_all, fetch_val


# ============================================================================
# User Queries
# ============================================================================

async def create_user(email: str, name: Optional[str] = None, image: Optional[str] = None) -> Dict[str, Any]:
    """Create a new user."""
    query = """
        INSERT INTO users (email, name, image)
        VALUES ($1, $2, $3)
        RETURNING id, email, name, email_verified, image, created_at, updated_at
    """
    row = await fetch_one(query, email, name, image)
    return dict(row) if row else None


async def get_user_by_id(user_id: UUID) -> Optional[Dict[str, Any]]:
    """Get user by ID."""
    query = "SELECT * FROM users WHERE id = $1"
    row = await fetch_one(query, user_id)
    return dict(row) if row else None


async def get_user_by_email(email: str) -> Optional[Dict[str, Any]]:
    """Get user by email."""
    query = "SELECT * FROM users WHERE email = $1"
    row = await fetch_one(query, email)
    return dict(row) if row else None


async def update_user(user_id: UUID, **fields) -> Optional[Dict[str, Any]]:
    """Update user fields."""
    set_clause = ", ".join([f"{key} = ${i+2}" for i, key in enumerate(fields.keys())])
    query = f"UPDATE users SET {set_clause} WHERE id = $1 RETURNING *"
    row = await fetch_one(query, user_id, *fields.values())
    return dict(row) if row else None


# ============================================================================
# Session Queries
# ============================================================================

async def create_session(user_id: UUID, token: str, expires_at: datetime,
                        ip_address: Optional[str] = None, user_agent: Optional[str] = None) -> Dict[str, Any]:
    """Create a new session."""
    query = """
        INSERT INTO sessions (user_id, token, expires_at, ip_address, user_agent)
        VALUES ($1, $2, $3, $4, $5)
        RETURNING id, user_id, token, expires_at, ip_address, user_agent, created_at
    """
    row = await fetch_one(query, user_id, token, expires_at, ip_address, user_agent)
    return dict(row) if row else None


async def get_session_by_token(token: str) -> Optional[Dict[str, Any]]:
    """Get session by token."""
    query = "SELECT * FROM sessions WHERE token = $1 AND expires_at > NOW()"
    row = await fetch_one(query, token)
    return dict(row) if row else None


async def delete_session(token: str) -> bool:
    """Delete a session."""
    query = "DELETE FROM sessions WHERE token = $1"
    result = await execute_query(query, token)
    return result == "DELETE 1"


async def delete_expired_sessions() -> int:
    """Delete all expired sessions."""
    query = "DELETE FROM sessions WHERE expires_at < NOW()"
    result = await execute_query(query)
    return int(result.split()[-1])


# ============================================================================
# User Preferences Queries
# ============================================================================

async def create_user_preferences(user_id: UUID, software_experience: str,
                                  hardware_experience: str, preferred_language: str = 'en') -> Dict[str, Any]:
    """Create user preferences."""
    query = """
        INSERT INTO user_preferences (user_id, software_experience, hardware_experience, preferred_language)
        VALUES ($1, $2, $3, $4)
        RETURNING *
    """
    row = await fetch_one(query, user_id, software_experience, hardware_experience, preferred_language)
    return dict(row) if row else None


async def get_user_preferences(user_id: UUID) -> Optional[Dict[str, Any]]:
    """Get user preferences."""
    query = "SELECT * FROM user_preferences WHERE user_id = $1"
    row = await fetch_one(query, user_id)
    return dict(row) if row else None


async def update_user_preferences(user_id: UUID, **fields) -> Optional[Dict[str, Any]]:
    """Update user preferences."""
    set_clause = ", ".join([f"{key} = ${i+2}" for i, key in enumerate(fields.keys())])
    query = f"UPDATE user_preferences SET {set_clause} WHERE user_id = $1 RETURNING *"
    row = await fetch_one(query, user_id, *fields.values())
    return dict(row) if row else None


# ============================================================================
# Chapter Queries
# ============================================================================

async def create_chapter(slug: str, title: str, module: str, week: int,
                        order_index: int, file_path: str) -> Dict[str, Any]:
    """Create a new chapter."""
    query = """
        INSERT INTO chapters (slug, title, module, week, order_index, file_path)
        VALUES ($1, $2, $3, $4, $5, $6)
        RETURNING *
    """
    row = await fetch_one(query, slug, title, module, week, order_index, file_path)
    return dict(row) if row else None


async def get_chapter_by_slug(slug: str) -> Optional[Dict[str, Any]]:
    """Get chapter by slug."""
    query = "SELECT * FROM chapters WHERE slug = $1"
    row = await fetch_one(query, slug)
    return dict(row) if row else None


async def get_chapters_by_module(module: str) -> List[Dict[str, Any]]:
    """Get all chapters in a module."""
    query = "SELECT * FROM chapters WHERE module = $1 ORDER BY order_index"
    rows = await fetch_all(query, module)
    return [dict(row) for row in rows]


async def get_all_chapters() -> List[Dict[str, Any]]:
    """Get all chapters."""
    query = "SELECT * FROM chapters ORDER BY module, order_index"
    rows = await fetch_all(query)
    return [dict(row) for row in rows]


# ============================================================================
# Content Chunk Queries
# ============================================================================

async def create_content_chunk(chapter_id: UUID, chunk_index: int, content: str,
                               token_count: int, qdrant_point_id: UUID) -> Dict[str, Any]:
    """Create a content chunk."""
    query = """
        INSERT INTO content_chunks (chapter_id, chunk_index, content, token_count, qdrant_point_id)
        VALUES ($1, $2, $3, $4, $5)
        RETURNING *
    """
    row = await fetch_one(query, chapter_id, chunk_index, content, token_count, qdrant_point_id)
    return dict(row) if row else None


async def get_chunks_by_chapter(chapter_id: UUID) -> List[Dict[str, Any]]:
    """Get all chunks for a chapter."""
    query = "SELECT * FROM content_chunks WHERE chapter_id = $1 ORDER BY chunk_index"
    rows = await fetch_all(query, chapter_id)
    return [dict(row) for row in rows]


async def get_chunk_by_qdrant_id(qdrant_point_id: UUID) -> Optional[Dict[str, Any]]:
    """Get chunk by Qdrant point ID."""
    query = "SELECT * FROM content_chunks WHERE qdrant_point_id = $1"
    row = await fetch_one(query, qdrant_point_id)
    return dict(row) if row else None


# ============================================================================
# Chat Message Queries
# ============================================================================

async def create_chat_message(session_id: str, role: str, content: str,
                              user_id: Optional[UUID] = None,
                              chapter_context: Optional[UUID] = None,
                              selected_text: Optional[str] = None) -> Dict[str, Any]:
    """Create a chat message."""
    query = """
        INSERT INTO chat_messages (user_id, session_id, role, content, chapter_context, selected_text)
        VALUES ($1, $2, $3, $4, $5, $6)
        RETURNING *
    """
    row = await fetch_one(query, user_id, session_id, role, content, chapter_context, selected_text)
    return dict(row) if row else None


async def get_chat_history(session_id: str, limit: int = 50) -> List[Dict[str, Any]]:
    """Get chat history for a session."""
    query = """
        SELECT * FROM chat_messages
        WHERE session_id = $1
        ORDER BY created_at DESC
        LIMIT $2
    """
    rows = await fetch_all(query, session_id, limit)
    return [dict(row) for row in reversed(rows)]


async def get_user_chat_history(user_id: UUID, limit: int = 100) -> List[Dict[str, Any]]:
    """Get all chat history for a user."""
    query = """
        SELECT * FROM chat_messages
        WHERE user_id = $1
        ORDER BY created_at DESC
        LIMIT $2
    """
    rows = await fetch_all(query, user_id, limit)
    return [dict(row) for row in rows]


async def delete_old_messages(days: int = 30) -> int:
    """Delete chat messages older than specified days."""
    query = "DELETE FROM chat_messages WHERE created_at < NOW() - INTERVAL '%s days'"
    result = await execute_query(query, days)
    return int(result.split()[-1])
