"""
Script to embed all textbook content into Qdrant vector database
Run this once to populate the database with textbook chapters
"""

import os
import sys
import asyncio
from pathlib import Path
from typing import List, Dict
import re
from uuid import uuid4
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Load environment variables
load_dotenv()

from api.services.openai_client import generate_embedding
from api.services.qdrant_client import init_collection, upsert_embeddings_batch


async def read_markdown_files(docs_dir: str = "docs") -> List[Dict]:
    """
    Read all markdown files from docs directory
    """
    # Get absolute path relative to script location
    script_dir = Path(__file__).parent.parent.parent
    docs_path = script_dir / docs_dir

    print(f"  Looking for markdown files in: {docs_path}")
    print(f"  Path exists: {docs_path.exists()}")

    markdown_files = []

    for md_file in docs_path.rglob("*.md"):
        # Skip tutorial files
        if "tutorial" in str(md_file):
            continue

        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract metadata from path
        parts = md_file.parts
        module = parts[-3] if len(parts) > 2 else "unknown"

        # Extract title from first heading
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        title = title_match.group(1) if title_match else md_file.stem

        # Create slug from file path
        slug = str(md_file.relative_to(docs_path)).replace('\\', '/').replace('.md', '')

        markdown_files.append({
            "file_path": str(md_file),
            "module": module,
            "title": title,
            "slug": slug,
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
    print("=" * 60)
    print("Starting content embedding process...")
    print("=" * 60)

    # Initialize collection
    print("\n[1/4] Initializing Qdrant collection...")
    await init_collection()

    # Read all markdown files
    print("\n[2/4] Reading markdown files...")
    markdown_files = await read_markdown_files()
    print(f"Found {len(markdown_files)} markdown files")

    # Process each file
    print("\n[3/4] Processing and embedding content...")
    total_chunks = 0
    all_points = []

    for idx, file_data in enumerate(markdown_files):
        print(f"\n  [{idx+1}/{len(markdown_files)}] {file_data['title']}")

        # Chunk the content
        chunks = chunk_content(file_data['content'])
        print(f"    - Created {len(chunks)} chunks")

        # Generate embeddings for each chunk
        chapter_id = uuid4()
        for chunk_idx, chunk in enumerate(chunks):
            # Generate embedding
            embedding = await generate_embedding(chunk)

            # Prepare point data
            point_data = {
                "point_id": uuid4(),
                "embedding": embedding,
                "chapter_id": chapter_id,
                "chapter_slug": file_data['slug'],
                "chapter_title": file_data['title'],
                "chunk_index": chunk_idx,
                "content": chunk,
                "token_count": len(chunk.split())
            }

            all_points.append(point_data)
            total_chunks += 1

            if total_chunks % 10 == 0:
                print(f"    - Processed {total_chunks} chunks...")

    # Batch upsert all points in smaller batches to avoid timeout
    print(f"\n[4/4] Uploading {total_chunks} embeddings to Qdrant...")
    if all_points:
        batch_size = 50
        for i in range(0, len(all_points), batch_size):
            batch = all_points[i:i+batch_size]
            await upsert_embeddings_batch(batch)
            print(f"  Uploaded batch {i//batch_size + 1}/{(len(all_points) + batch_size - 1)//batch_size} ({len(batch)} points)")
    else:
        print("  WARNING: No points to upload!")

    print("\n" + "=" * 60)
    print("Embedding complete!")
    print("=" * 60)
    print(f"Total files processed: {len(markdown_files)}")
    print(f"Total chunks embedded: {total_chunks}")
    print(f"Collection name: textbook_embeddings")
    print("=" * 60)

    return {
        "total_files": len(markdown_files),
        "total_chunks": total_chunks,
        "collection_name": "textbook_embeddings"
    }


if __name__ == "__main__":
    asyncio.run(embed_all_content())
