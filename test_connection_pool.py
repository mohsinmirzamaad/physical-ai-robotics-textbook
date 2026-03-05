"""Test the connection pool module"""
import asyncio
import os
import sys
from pathlib import Path

# Add api directory to path
sys.path.insert(0, str(Path(__file__).parent / 'api'))

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

from dotenv import load_dotenv
load_dotenv()

async def test_pool():
    print("=" * 60)
    print("Testing API Connection Pool Module")
    print("=" * 60)

    try:
        from database.connection import get_pool, close_pool, fetch_one, fetch_all, execute_query

        print("\n1. Testing get_pool()...")
        pool = await get_pool()
        print(f"   OK: Pool created")
        print(f"   - Min size: {pool._minsize}")
        print(f"   - Max size: {pool._maxsize}")

        print("\n2. Testing fetch_one()...")
        result = await fetch_one("SELECT 1 as test")
        print(f"   OK: Result = {dict(result)}")

        print("\n3. Testing fetch_all()...")
        tables = await fetch_all("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema='public'
            ORDER BY table_name
        """)
        print(f"   OK: Found {len(tables)} tables")
        for t in tables:
            print(f"      - {t['table_name']}")

        print("\n4. Testing execute_query()...")
        # Test with a safe query that doesn't modify data
        result = await execute_query("SELECT NOW()")
        print(f"   OK: Query executed")

        print("\n5. Testing close_pool()...")
        await close_pool()
        print(f"   OK: Pool closed")

        print("\n" + "=" * 60)
        print("All connection pool tests passed!")
        print("=" * 60)
        return True

    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_pool())
    sys.exit(0 if success else 1)
