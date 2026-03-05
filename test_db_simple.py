"""Simple database connection test"""
import asyncio
import os
import sys
from dotenv import load_dotenv
import asyncpg

# Fix Windows encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

load_dotenv()

async def test():
    print("=" * 60)
    print("Testing Neon Postgres Connection")
    print("=" * 60)

    db_url = os.getenv("DATABASE_URL")
    if not db_url:
        print("ERROR: DATABASE_URL not found")
        return

    print(f"\nDatabase URL: {db_url[:30]}...{db_url[-20:]}")

    try:
        print("\nConnecting to database...")
        conn = await asyncpg.connect(db_url)
        print("SUCCESS: Connected to Neon database!")

        print("\nRunning SELECT 1...")
        result = await conn.fetchval("SELECT 1")
        print(f"Result: {result}")

        print("\nChecking PostgreSQL version...")
        version = await conn.fetchval("SELECT version()")
        print(f"Version: {version.split(',')[0]}")

        print("\nListing tables...")
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema='public'
            ORDER BY table_name
        """)

        if tables:
            print(f"Found {len(tables)} tables:")
            for t in tables:
                print(f"  - {t['table_name']}")
        else:
            print("No tables found")

        print("\nChecking for Better-Auth tables...")
        table_names = [t['table_name'] for t in tables]

        for required in ['users', 'sessions']:
            if required in table_names:
                print(f"  OK: {required} exists")
            else:
                print(f"  MISSING: {required}")

        if 'users' in table_names:
            print("\nUsers table schema:")
            cols = await conn.fetch("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name='users'
                ORDER BY ordinal_position
            """)
            for c in cols:
                print(f"  - {c['column_name']}: {c['data_type']}")

        print("\nTable row counts:")
        for table in table_names:
            count = await conn.fetchval(f"SELECT COUNT(*) FROM {table}")
            print(f"  - {table}: {count} rows")

        await conn.close()
        print("\n" + "=" * 60)
        print("All tests completed successfully!")
        print("=" * 60)

    except Exception as e:
        print(f"\nERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test())
