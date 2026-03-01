"""
Qdrant client service for vector database operations
"""

import os
from typing import List, Dict, Any, Optional
from uuid import UUID, uuid4
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest
)
from dotenv import load_dotenv

load_dotenv()

# Initialize Qdrant client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

COLLECTION_NAME = "textbook_embeddings"
VECTOR_SIZE = 1536  # text-embedding-3-small dimension


async def init_collection():
    """
    Initialize Qdrant collection if it doesn't exist.
    Should be called on application startup.
    """
    collections = client.get_collections().collections
    collection_names = [col.name for col in collections]

    if COLLECTION_NAME not in collection_names:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=VECTOR_SIZE,
                distance=Distance.COSINE
            )
        )
        print(f"Created Qdrant collection: {COLLECTION_NAME}")
    else:
        print(f"Qdrant collection already exists: {COLLECTION_NAME}")


async def upsert_embedding(
    point_id: UUID,
    embedding: List[float],
    chapter_id: UUID,
    chapter_slug: str,
    chapter_title: str,
    chunk_index: int,
    content: str,
    token_count: int
) -> bool:
    """
    Insert or update an embedding in Qdrant.

    Args:
        point_id: Unique ID for the point
        embedding: Vector embedding
        chapter_id: Chapter UUID
        chapter_slug: Chapter slug
        chapter_title: Chapter title
        chunk_index: Chunk index within chapter
        content: Text content
        token_count: Number of tokens

    Returns:
        True if successful
    """
    point = PointStruct(
        id=str(point_id),
        vector=embedding,
        payload={
            "chapter_id": str(chapter_id),
            "chapter_slug": chapter_slug,
            "chapter_title": chapter_title,
            "chunk_index": chunk_index,
            "content": content,
            "token_count": token_count
        }
    )

    client.upsert(
        collection_name=COLLECTION_NAME,
        points=[point]
    )

    return True


async def upsert_embeddings_batch(points: List[Dict[str, Any]]) -> bool:
    """
    Insert or update multiple embeddings in a single batch.

    Args:
        points: List of point dictionaries with keys:
            - point_id, embedding, chapter_id, chapter_slug, chapter_title,
              chunk_index, content, token_count

    Returns:
        True if successful
    """
    qdrant_points = []

    for point in points:
        qdrant_point = PointStruct(
            id=str(point["point_id"]),
            vector=point["embedding"],
            payload={
                "chapter_id": str(point["chapter_id"]),
                "chapter_slug": point["chapter_slug"],
                "chapter_title": point["chapter_title"],
                "chunk_index": point["chunk_index"],
                "content": point["content"],
                "token_count": point["token_count"]
            }
        )
        qdrant_points.append(qdrant_point)

    client.upsert(
        collection_name=COLLECTION_NAME,
        points=qdrant_points
    )

    return True


async def search_similar(
    query_embedding: List[float],
    limit: int = 5,
    chapter_filter: Optional[str] = None,
    score_threshold: float = 0.7
) -> List[Dict[str, Any]]:
    """
    Search for similar embeddings using vector similarity.

    Args:
        query_embedding: Query vector
        limit: Maximum number of results
        chapter_filter: Optional chapter slug to filter results
        score_threshold: Minimum similarity score (0-1)

    Returns:
        List of search results with content and metadata
    """
    search_filter = None
    if chapter_filter:
        search_filter = Filter(
            must=[
                FieldCondition(
                    key="chapter_slug",
                    match=MatchValue(value=chapter_filter)
                )
            ]
        )

    results = client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=limit,
        query_filter=search_filter,
        score_threshold=score_threshold
    )

    return [
        {
            "id": result.id,
            "score": result.score,
            "chapter_id": result.payload["chapter_id"],
            "chapter_slug": result.payload["chapter_slug"],
            "chapter_title": result.payload["chapter_title"],
            "chunk_index": result.payload["chunk_index"],
            "content": result.payload["content"],
            "token_count": result.payload["token_count"]
        }
        for result in results
    ]


async def get_point_by_id(point_id: UUID) -> Optional[Dict[str, Any]]:
    """
    Retrieve a specific point by ID.

    Args:
        point_id: Point UUID

    Returns:
        Point data or None if not found
    """
    points = client.retrieve(
        collection_name=COLLECTION_NAME,
        ids=[str(point_id)]
    )

    if not points:
        return None

    point = points[0]
    return {
        "id": point.id,
        "vector": point.vector,
        "chapter_id": point.payload["chapter_id"],
        "chapter_slug": point.payload["chapter_slug"],
        "chapter_title": point.payload["chapter_title"],
        "chunk_index": point.payload["chunk_index"],
        "content": point.payload["content"],
        "token_count": point.payload["token_count"]
    }


async def delete_chapter_embeddings(chapter_slug: str) -> bool:
    """
    Delete all embeddings for a specific chapter.

    Args:
        chapter_slug: Chapter slug

    Returns:
        True if successful
    """
    client.delete(
        collection_name=COLLECTION_NAME,
        points_selector=Filter(
            must=[
                FieldCondition(
                    key="chapter_slug",
                    match=MatchValue(value=chapter_slug)
                )
            ]
        )
    )

    return True


async def get_collection_info() -> Dict[str, Any]:
    """
    Get information about the collection.

    Returns:
        Collection statistics
    """
    info = client.get_collection(collection_name=COLLECTION_NAME)

    return {
        "name": COLLECTION_NAME,
        "vectors_count": info.vectors_count,
        "points_count": info.points_count,
        "status": info.status,
        "optimizer_status": info.optimizer_status
    }


async def health_check() -> bool:
    """
    Check if Qdrant is healthy and accessible.

    Returns:
        True if healthy
    """
    try:
        collections = client.get_collections()
        return True
    except Exception as e:
        print(f"Qdrant health check failed: {e}")
        return False
