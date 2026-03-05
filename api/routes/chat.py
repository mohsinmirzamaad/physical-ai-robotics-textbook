"""
Chat API routes for RAG chatbot
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import os

from services.openai_client import generate_embedding
from services.qdrant_client import client as qdrant_client
from database.queries import create_chat_message, get_chat_history
from openai import AsyncOpenAI

router = APIRouter()

# Initialize OpenAI client
openai_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class ChatRequest(BaseModel):
    message: str
    user_id: Optional[str] = None
    session_id: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    session_id: str


@router.post("/message", response_model=ChatResponse)
async def chat_message(request: ChatRequest):
    """
    Handle chat message with RAG
    """
    try:
        # Generate embedding for user query
        query_embedding = await generate_embedding(request.message)

        # Search for relevant content in Qdrant
        search_results = qdrant_client.query_points(
            collection_name="textbook_content",
            query=query_embedding,
            limit=5
        ).points

        # Extract context from search results
        context_chunks = []
        sources = []

        for result in search_results:
            context_chunks.append(result.payload.get("text", ""))
            sources.append({
                "chapter": result.payload.get("chapter_title", ""),
                "module": result.payload.get("module", ""),
                "score": result.score
            })

        # Generate response using OpenAI with context
        context_text = "\n\n".join(context_chunks)
        prompt = f"""You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook.

Context from the textbook:
{context_text}

User question: {request.message}

Please provide a helpful answer based on the context above. If the context doesn't contain relevant information, say so."""

        response = await openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook."},
                {"role": "user", "content": prompt}
            ]
        )

        answer = response.choices[0].message.content

        # Save chat message to database (optional - may fail if DB not set up)
        session_id = request.session_id or "anonymous"
        # Commenting out DB save for now
        # if request.user_id:
        #     await create_chat_message(
        #         user_id=request.user_id,
        #         message=request.message,
        #         response=answer,
        #         session_id=session_id
        #     )

        return ChatResponse(
            response=answer,
            sources=sources,
            session_id=session_id
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")


@router.get("/history/{user_id}")
async def get_history(user_id: str, limit: int = 50):
    """
    Get chat history for a user
    """
    try:
        history = await get_chat_history(user_id, limit)
        return {"history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching history: {str(e)}")
