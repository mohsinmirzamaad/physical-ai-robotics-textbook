"""
Content API routes for personalization and translation
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import os

from openai import AsyncOpenAI
from api.database.queries import get_user_preferences

router = APIRouter()

# Initialize OpenAI client
openai_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class PersonalizeRequest(BaseModel):
    content: str
    user_id: str
    chapter_title: str


class TranslateRequest(BaseModel):
    content: str
    target_language: str = "urdu"


class ContentResponse(BaseModel):
    content: str
    original_length: int
    processed_length: int


@router.post("/personalize", response_model=ContentResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize content based on user background
    """
    try:
        # For now, use a default background since DB might not be set up
        # Get user preferences (commented out for now)
        # preferences = await get_user_preferences(request.user_id)
        # if not preferences:
        #     raise HTTPException(status_code=404, detail="User preferences not found")

        user_background = "intermediate"  # Default

        # Personalize content using OpenAI
        prompt = f"""You are adapting educational content for a student with {user_background} level experience.

Original content:
{request.content}

Please rewrite this content to be appropriate for a {user_background} level student. Keep the same information but adjust the complexity, examples, and explanations accordingly."""

        response = await openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": f"You are an educational content adapter. Adapt content for {user_background} level students."},
                {"role": "user", "content": prompt}
            ]
        )

        personalized = response.choices[0].message.content

        return ContentResponse(
            content=personalized,
            original_length=len(request.content),
            processed_length=len(personalized)
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Personalization error: {str(e)}")


@router.post("/translate", response_model=ContentResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to target language
    """
    try:
        # Translate content using OpenAI
        prompt = f"""Translate the following English text to {request.target_language}. Maintain technical accuracy and formatting.

Text to translate:
{request.content}"""

        response = await openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": f"You are a professional translator. Translate technical content to {request.target_language}."},
                {"role": "user", "content": prompt}
            ]
        )

        translated = response.choices[0].message.content

        return ContentResponse(
            content=translated,
            original_length=len(request.content),
            processed_length=len(translated)
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation error: {str(e)}")


@router.post("/embed")
async def embed_content():
    """
    Embed all textbook content into Qdrant
    This should be run once to populate the vector database
    """
    try:
        from api.scripts.embed_content import embed_all_content

        result = await embed_all_content()
        return {
            "status": "success",
            "message": "Content embedded successfully",
            "details": result
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Embedding error: {str(e)}")
