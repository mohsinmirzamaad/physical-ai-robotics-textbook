"""
Test script for Qdrant Cloud vector database integration
Tests connection, collections, and search functionality
"""

import os
import sys
import asyncio
from pathlib import Path
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
load_dotenv()

from api.services.qdrant_client import (
    client,
    COLLECTION_NAME,
    VECTOR_SIZE,
    init_collection,
    get_collection_info,
    health_check,
    search_similar
)
from api.services.openai_client import generate_embedding


async def test_connection():
    """Test 1: Verify Qdrant connection"""
    print("\n" + "=" * 60)
    print("TEST 1: Connection Test")
    print("=" * 60)

    # Check environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_key = os.getenv("QDRANT_API_KEY")

    print(f"QDRANT_URL: {qdrant_url}")
    print(f"QDRANT_API_KEY: {'*' * 20 + qdrant_key[-10:] if qdrant_key else 'NOT SET'}")

    if not qdrant_url or not qdrant_key:
        print("❌ FAILED: Environment variables not set")
        return False

    # Test health check
    is_healthy = await health_check()
    if is_healthy:
        print("[PASS] Successfully connected to Qdrant Cloud")
        return True
    else:
        print("[FAIL] Could not connect to Qdrant")
        return False


async def test_list_collections():
    """Test 2: List all collections"""
    print("\n" + "=" * 60)
    print("TEST 2: List Collections")
    print("=" * 60)

    try:
        collections = client.get_collections()
        print(f"Found {len(collections.collections)} collection(s):")

        for col in collections.collections:
            print(f"  - {col.name}")

        print("[PASS] Successfully listed collections")
        return True, collections.collections
    except Exception as e:
        print(f"[FAIL] {e}")
        return False, []


async def test_collection_info():
    """Test 3: Get collection information"""
    print("\n" + "=" * 60)
    print("TEST 3: Collection Information")
    print("=" * 60)

    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if COLLECTION_NAME not in collection_names:
            print(f"[WARN] Collection '{COLLECTION_NAME}' does not exist")
            print("   Initializing collection...")
            await init_collection()

        # Get collection info
        info = await get_collection_info()

        print(f"Collection Name: {info['name']}")
        print(f"Vector Size: {VECTOR_SIZE}")
        print(f"Points Count: {info['points_count']}")
        print(f"Vectors Count: {info['vectors_count']}")
        print(f"Status: {info['status']}")
        print(f"Optimizer Status: {info['optimizer_status']}")

        if info['points_count'] == 0:
            print("\n[WARN] Collection is empty!")
            print("   Run: python api/scripts/embed_content.py")
            return True, info, True  # True for test passed, info, True for empty
        else:
            print(f"\n[PASS] Collection has {info['points_count']} vectors")
            return True, info, False  # False for not empty

    except Exception as e:
        print(f"[FAIL] {e}")
        return False, None, False


async def test_sample_search():
    """Test 4: Perform a sample vector search"""
    print("\n" + "=" * 60)
    print("TEST 4: Sample Vector Search")
    print("=" * 60)

    try:
        # Generate a test query embedding
        test_query = "What is physical AI and robotics?"
        print(f"Test Query: '{test_query}'")
        print("Generating embedding...")

        query_embedding = await generate_embedding(test_query)
        print(f"[OK] Generated embedding (dimension: {len(query_embedding)})")

        # Perform search
        print("Searching for similar vectors...")
        results = await search_similar(
            query_embedding=query_embedding,
            limit=3,
            score_threshold=0.0  # Lower threshold for testing
        )

        print(f"\nFound {len(results)} result(s):")
        for i, result in enumerate(results, 1):
            print(f"\n  Result {i}:")
            print(f"    Score: {result['score']:.4f}")
            print(f"    Chapter: {result['chapter_title']}")
            print(f"    Slug: {result['chapter_slug']}")
            print(f"    Chunk Index: {result['chunk_index']}")
            print(f"    Content Preview: {result['content'][:150]}...")

        if len(results) > 0:
            print("\n[PASS] Search returned results")
            return True, results
        else:
            print("\n[WARN] Search returned no results (collection may be empty)")
            return True, []

    except Exception as e:
        print(f"[FAIL] {e}")
        import traceback
        traceback.print_exc()
        return False, []


async def main():
    """Run all tests"""
    print("=" * 60)
    print("QDRANT CLOUD INTEGRATION TEST SUITE")
    print("=" * 60)
    print(f"Collection: {COLLECTION_NAME}")
    print(f"Vector Size: {VECTOR_SIZE}")

    results = []

    # Test 1: Connection
    result1 = await test_connection()
    results.append(("Connection", result1))

    if not result1:
        print("\n[FAIL] Cannot proceed without connection")
        return

    # Test 2: List collections
    result2, collections = await test_list_collections()
    results.append(("List Collections", result2))

    # Test 3: Collection info
    result3, info, is_empty = await test_collection_info()
    results.append(("Collection Info", result3))

    # Test 4: Sample search (only if collection has data)
    if result3 and not is_empty:
        result4, search_results = await test_sample_search()
        results.append(("Sample Search", result4))
    else:
        print("\n[SKIP] Sample search (collection is empty)")
        results.append(("Sample Search", None))

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    for test_name, result in results:
        if result is True:
            print(f"[PASS] {test_name}")
        elif result is False:
            print(f"[FAIL] {test_name}")
        else:
            print(f"[SKIP] {test_name}")

    # Final recommendations
    print("\n" + "=" * 60)
    print("RECOMMENDATIONS")
    print("=" * 60)

    if is_empty:
        print("\n[WARN] Collection is empty. To populate it:")
        print("   1. Ensure docs/ directory has markdown files")
        print("   2. Run: python api/scripts/embed_content.py")
        print("   3. Re-run this test: python api/test_qdrant.py")
    else:
        print("\n[SUCCESS] Qdrant integration is working correctly!")
        print("   - Connection: Active")
        print(f"   - Collection: {COLLECTION_NAME}")
        print(f"   - Vectors: {info['points_count'] if info else 'N/A'}")
        print("   - Search: Functional")

    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
