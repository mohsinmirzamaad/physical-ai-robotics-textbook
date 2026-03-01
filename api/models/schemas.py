"""
Pydantic schemas for request/response models
"""

from pydantic import BaseModel, EmailStr, Field, validator
from typing import Optional, List, Literal
from datetime import datetime
from uuid import UUID


# ============================================================================
# Authentication Schemas
# ============================================================================

class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: Optional[str] = None
    software_experience: Literal["beginner", "intermediate", "advanced"] = "beginner"
    hardware_experience: Literal["beginner", "intermediate", "advanced"] = "beginner"


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    id: UUID
    email: str
    name: Optional[str]
    email_verified: bool
    created_at: datetime


class SessionResponse(BaseModel):
    user: UserResponse
    session_id: str
    expires_at: datetime


# ============================================================================
# User Preferences Schemas
# ============================================================================

class UserPreferencesRequest(BaseModel):
    software_experience: Literal["beginner", "intermediate", "advanced"]
    hardware_experience: Literal["beginner", "intermediate", "advanced"]
    preferred_language: Literal["en", "ur"] = "en"


class UserPreferencesResponse(BaseModel):
    id: UUID
    user_id: UUID
    software_experience: str
    hardware_experience: str
    preferred_language: str
    created_at: datetime
    updated_at: datetime


# ============================================================================
# Chatbot Schemas
# ============================================================================

class ChatMessage(BaseModel):
    role: Literal["user", "assistant", "system"]
    content: str


class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str
    chapter_slug: Optional[str] = None
    selected_text: Optional[str] = None
    history: Optional[List[ChatMessage]] = []


class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    session_id: str


class ChatStreamChunk(BaseModel):
    type: Literal["token", "source", "done"]
    content: Optional[str] = None
    sources: Optional[List[dict]] = None


# ============================================================================
# Content Schemas
# ============================================================================

class ChapterMetadata(BaseModel):
    id: UUID
    slug: str
    title: str
    module: str
    week: int
    order_index: int
    file_path: str


class ContentChunk(BaseModel):
    id: UUID
    chapter_id: UUID
    chunk_index: int
    content: str
    token_count: int


class PersonalizeRequest(BaseModel):
    chapter_slug: str
    experience_level: Literal["beginner", "intermediate", "advanced"]


class PersonalizeResponse(BaseModel):
    original_content: str
    personalized_content: str
    experience_level: str
    cached: bool = False


class TranslateRequest(BaseModel):
    chapter_slug: str
    target_language: Literal["en", "ur"]


class TranslateResponse(BaseModel):
    original_content: str
    translated_content: str
    target_language: str
    cached: bool = False


# ============================================================================
# Embedding Schemas
# ============================================================================

class EmbeddingRequest(BaseModel):
    text: str
    model: str = "text-embedding-3-small"


class EmbeddingResponse(BaseModel):
    embedding: List[float]
    model: str
    token_count: int


# ============================================================================
# Search Schemas
# ============================================================================

class SearchRequest(BaseModel):
    query: str
    limit: int = Field(default=5, ge=1, le=20)
    chapter_filter: Optional[str] = None


class SearchResult(BaseModel):
    chunk_id: UUID
    chapter_slug: str
    chapter_title: str
    content: str
    score: float


class SearchResponse(BaseModel):
    results: List[SearchResult]
    query: str
    total_results: int


# ============================================================================
# Health Check Schema
# ============================================================================

class HealthResponse(BaseModel):
    status: str
    service: str
    version: str
    database: Optional[str] = None
    qdrant: Optional[str] = None
    openai: Optional[str] = None


# ============================================================================
# Error Schemas
# ============================================================================

class ErrorResponse(BaseModel):
    error: str
    detail: Optional[str] = None
    status_code: int
