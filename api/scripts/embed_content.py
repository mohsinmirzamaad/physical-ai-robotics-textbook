"""
Script to embed all textbook content into Qdrant vector database
Run this once to populate the database with textbook chapters
"""

import os
import asyncio
from pathlib import Path
from typing import List, Dict
import re

from api.services.openai_client import OpenAIClient
from api.services.qdrant_client import QdrantClient


async def read_markdown_files(docs_dir: str = "docs") -> List[Dict]:
    """
    Read all markdown files from docs directory
    """
    docs_path = Path(docs_dir)
    markdown_files = []

    for md_file in docs_path.rglob("*.md"):
        if md_file.name == "intro.md":
            continue

        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract metadata from path
        parts = md_file.parts
        module = parts[1] if len(parts) > 1 else "unknown"

        # Extract title from first heading
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        title = title_match.group(1) if title_match else md_file.stem

        markdown_files.append({
            "file_path": str(md_file),
            "module": module,
            "title": title,
            "content": content
        })

    return markdown_files


def chunk_content(content: str, chunk_size: int = 1000) -> List[str]:
    """
    Split content into chunks by paragraphs
    """
    # Split by double newlines (paragraphs)
    paragraphs = content.split('\n\n')

    chunks = []
    current_chunk = ""

    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"

    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks


async def embed_all_content():
    """
    Main function to embed all textbook content
    """
    print("Starting content embedding process...")

    # Initialize clients
    openai_client = OpenAIClient(api_key=os.getenv("OPENAI_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Create collection if it doesn't exist
    collection_name = "textbook_content"
    try:
        await qdrant_client.create_collection(
            collection_name=collection_name,
            vector_size=1536  # OpenAI embedding size
        )
        print(f"Collection '{collection_name}' created")
    except Exception as e:
        print(f"Collection might already exist: {e}")

    # Read all markdown files
    print("Reading markdown files...")
    markdown_files = await read_markdown_files()
    print(f"Found {len(markdown_files)} markdown files")

    # Process each file
    total_chunks = 0
    for idx, file_data in enumerate(markdown_files):
        print(f"\nProcessing {idx+1}/{len(markdown_files)}: {file_data['title']}")

        # Chunk the content
        chunks = chunk_content(file_data['content'])
        print(f"  Created {len(chunks)} chunks")

        # Generate embeddings and store
        for chunk_idx, chunk in enumerate(chunks):
            # Generate embedding
            embedding = await openai_client.generate_embedding(chunk)

            # Prepare metadata
            metadata = {
                "text": chunk,
                "chapter_title": file_data['title'],
                "module": file_data['module'],
                "file_path": file_data['file_path'],
                "chunk_index": chunk_idx
            }

            # Store in Qdrant
            point_id = f"{idx}_{chunk_idx}"
            await qdrant_client.upsert(
                collection_name=collection_name,
                points=[{
                    "id": point_id,
                    "vector": embedding,
                    "payload": metadata
                }]
            )

            total_chunks += 1
            if total_chunks % 10 == 0:
                print(f"  Processed {total_chunks} chunks...")

    print(f"\n✅ Embedding complete! Total chunks: {total_chunks}")

    return {
        "total_files": len(markdown_files),
        "total_chunks": total_chunks,
        "collection_name": collection_name
    }


if __name__ == "__main__":
    asyncio.run(embed_all_content())
