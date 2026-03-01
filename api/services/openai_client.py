"""
OpenAI client service for embeddings and chat completions
"""

import os
from typing import List, Dict, Any, AsyncGenerator
from openai import AsyncOpenAI
from dotenv import load_dotenv

load_dotenv()

# Initialize OpenAI client
client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))


async def generate_embedding(text: str, model: str = "text-embedding-3-small") -> List[float]:
    """
    Generate embedding for text using OpenAI API.

    Args:
        text: Text to embed
        model: Embedding model to use

    Returns:
        List of floats representing the embedding vector
    """
    response = await client.embeddings.create(
        input=text,
        model=model
    )
    return response.data[0].embedding


async def generate_embeddings_batch(texts: List[str], model: str = "text-embedding-3-small") -> List[List[float]]:
    """
    Generate embeddings for multiple texts in a single API call.

    Args:
        texts: List of texts to embed
        model: Embedding model to use

    Returns:
        List of embedding vectors
    """
    response = await client.embeddings.create(
        input=texts,
        model=model
    )
    return [item.embedding for item in response.data]


async def chat_completion(
    messages: List[Dict[str, str]],
    model: str = "gpt-4o",
    temperature: float = 0.7,
    max_tokens: int = 1000
) -> str:
    """
    Generate chat completion using OpenAI API.

    Args:
        messages: List of message dicts with 'role' and 'content'
        model: Chat model to use
        temperature: Sampling temperature
        max_tokens: Maximum tokens to generate

    Returns:
        Generated response text
    """
    response = await client.chat.completions.create(
        model=model,
        messages=messages,
        temperature=temperature,
        max_tokens=max_tokens
    )
    return response.choices[0].message.content


async def chat_completion_stream(
    messages: List[Dict[str, str]],
    model: str = "gpt-4o",
    temperature: float = 0.7,
    max_tokens: int = 1000
) -> AsyncGenerator[str, None]:
    """
    Generate streaming chat completion using OpenAI API.

    Args:
        messages: List of message dicts with 'role' and 'content'
        model: Chat model to use
        temperature: Sampling temperature
        max_tokens: Maximum tokens to generate

    Yields:
        Token strings as they are generated
    """
    stream = await client.chat.completions.create(
        model=model,
        messages=messages,
        temperature=temperature,
        max_tokens=max_tokens,
        stream=True
    )

    async for chunk in stream:
        if chunk.choices[0].delta.content is not None:
            yield chunk.choices[0].delta.content


async def generate_rag_response(
    query: str,
    context_chunks: List[str],
    chat_history: List[Dict[str, str]] = None,
    model: str = "gpt-4o"
) -> str:
    """
    Generate RAG response using retrieved context chunks.

    Args:
        query: User query
        context_chunks: Retrieved context chunks from vector search
        chat_history: Previous chat messages for context
        model: Chat model to use

    Returns:
        Generated response text
    """
    # Build context from chunks
    context = "\n\n".join([f"[Context {i+1}]\n{chunk}" for i, chunk in enumerate(context_chunks)])

    # Build system message
    system_message = f"""You are an expert AI tutor for Physical AI and Humanoid Robotics.
Your role is to help students learn about ROS 2, simulation environments (Gazebo, Unity, NVIDIA Isaac),
and Vision-Language-Action models for humanoid robots.

Use the following context from the textbook to answer the student's question.
If the context doesn't contain enough information, say so and provide general guidance.
Always be clear, educational, and encourage hands-on learning.

Context:
{context}
"""

    # Build messages
    messages = [{"role": "system", "content": system_message}]

    # Add chat history if provided
    if chat_history:
        messages.extend(chat_history[-10:])  # Last 10 messages for context

    # Add current query
    messages.append({"role": "user", "content": query})

    return await chat_completion(messages, model=model, temperature=0.7, max_tokens=1000)


async def generate_rag_response_stream(
    query: str,
    context_chunks: List[str],
    chat_history: List[Dict[str, str]] = None,
    model: str = "gpt-4o"
) -> AsyncGenerator[str, None]:
    """
    Generate streaming RAG response using retrieved context chunks.

    Args:
        query: User query
        context_chunks: Retrieved context chunks from vector search
        chat_history: Previous chat messages for context
        model: Chat model to use

    Yields:
        Token strings as they are generated
    """
    # Build context from chunks
    context = "\n\n".join([f"[Context {i+1}]\n{chunk}" for i, chunk in enumerate(context_chunks)])

    # Build system message
    system_message = f"""You are an expert AI tutor for Physical AI and Humanoid Robotics.
Your role is to help students learn about ROS 2, simulation environments (Gazebo, Unity, NVIDIA Isaac),
and Vision-Language-Action models for humanoid robots.

Use the following context from the textbook to answer the student's question.
If the context doesn't contain enough information, say so and provide general guidance.
Always be clear, educational, and encourage hands-on learning.

Context:
{context}
"""

    # Build messages
    messages = [{"role": "system", "content": system_message}]

    # Add chat history if provided
    if chat_history:
        messages.extend(chat_history[-10:])  # Last 10 messages for context

    # Add current query
    messages.append({"role": "user", "content": query})

    async for token in chat_completion_stream(messages, model=model, temperature=0.7, max_tokens=1000):
        yield token


async def personalize_content(
    content: str,
    experience_level: str,
    model: str = "gpt-4o"
) -> str:
    """
    Personalize content based on user experience level.

    Args:
        content: Original content
        experience_level: User's experience level (beginner/intermediate/advanced)
        model: Chat model to use

    Returns:
        Personalized content
    """
    prompts = {
        "beginner": "Simplify this content for absolute beginners. Add more explanations, examples, and step-by-step guidance. Avoid jargon.",
        "intermediate": "Adapt this content for intermediate learners. Balance explanations with practical examples. Include some technical details.",
        "advanced": "Enhance this content for advanced learners. Add technical depth, edge cases, and advanced concepts. Assume strong foundation."
    }

    messages = [
        {"role": "system", "content": f"{prompts[experience_level]}\n\nMaintain the same structure and code examples."},
        {"role": "user", "content": content}
    ]

    return await chat_completion(messages, model=model, temperature=0.5, max_tokens=2000)


async def translate_content(
    content: str,
    target_language: str,
    model: str = "gpt-4o"
) -> str:
    """
    Translate content to target language.

    Args:
        content: Original content in English
        target_language: Target language code (e.g., 'ur' for Urdu)
        model: Chat model to use

    Returns:
        Translated content
    """
    language_names = {
        "ur": "Urdu",
        "en": "English"
    }

    messages = [
        {
            "role": "system",
            "content": f"Translate the following technical content to {language_names[target_language]}. "
                      f"Preserve code blocks, technical terms (ROS 2, Gazebo, etc.), and markdown formatting. "
                      f"Ensure technical accuracy and natural language flow."
        },
        {"role": "user", "content": content}
    ]

    return await chat_completion(messages, model=model, temperature=0.3, max_tokens=3000)
