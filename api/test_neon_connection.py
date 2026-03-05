"""
Test script to verify Neon Serverless Postgres database connection.
Tests connectivity, lists tables, and verifies Better-Auth schema.
"""

import asyncio
import os
import sys
from pathlib import Path

# Fix Windows console encoding for emojis
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
import asyncpg

# Load environment variables
load_dotenv()


async def test_connection():
    """Test basic database connectivity."""
    print("\n" + "=" * 60)
    print("🔍 Testing Neon Postgres Connection")
    print("=" * 60)

    database_url = os.getenv("DATABASE_URL")

    if not database_url:
        print("❌ DATABASE_URL environment variable is not set")
        return False

    print(f"\n✅ DATABASE_URL found")
    print(f"   Connection string: {database_url[:30]}...{database_url[-20:]}")

    try:
        # Test connection
        print("\n📡 Attempting to connect to Neon database...")
        conn = await asyncpg.connect(database_url)
        print("✅ Successfully connected to Neon database!")

        # Test simple query
        print("\n🔍 Running SELECT 1 test...")
        result = await conn.fetchval("SELECT 1")
        if result == 1:
            print("✅ SELECT 1 test passed")
        else:
            print(f"❌ SELECT 1 returned unexpected value: {result}")
            await conn.close()
            return False

        # Get database version
        print("\n🔍 Checking PostgreSQL version...")
        version = await conn.fetchval("SELECT version()")
        print(f"✅ PostgreSQL version: {version.split(',')[0]}")

        # List all tables
        print("\n🔍 Listing all tables in public schema...")
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema='public'
            ORDER BY table_name
        """)

        if tables:
            print(f"✅ Found {len(tables)} table(s):")
            for table in tables:
                print(f"   - {table['table_name']}")
        else:
            print("⚠️  No tables found in public schema")

        # Check for Better-Auth required tables
        print("\n🔍 Checking Better-Auth schema...")
        required_tables = ['users', 'sessions']
        table_names = [t['table_name'] for t in tables]

        missing_tables = []
        for required_table in required_tables:
            if required_table in table_names:
                print(f"✅ Table '{required_table}' exists")
            else:
                print(f"❌ Table '{required_table}' is missing")
                missing_tables.append(required_table)

        # Check optional tables
        optional_tables = ['user_preferences', 'chapters', 'content_chunks', 'chat_messages']
        for optional_table in optional_tables:
            if optional_table in table_names:
                print(f"✅ Table '{optional_table}' exists")
            else:
                print(f"⚠️  Table '{optional_table}' not found (optional)")

        # If tables exist, check their schema
        if 'users' in table_names:
            print("\n🔍 Checking 'users' table schema...")
            columns = await conn.fetch("""
                SELECT column_name, data_type, is_nullable
                FROM information_schema.columns
                WHERE table_schema='public' AND table_name='users'
                ORDER BY ordinal_position
            """)
            print(f"✅ Users table has {len(columns)} columns:")
            for col in columns:
                nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
                print(f"   - {col['column_name']}: {col['data_type']} ({nullable})")

        if 'sessions' in table_names:
            print("\n🔍 Checking 'sessions' table schema...")
            columns = await conn.fetch("""
                SELECT column_name, data_type, is_nullable
                FROM information_schema.columns
                WHERE table_schema='public' AND table_name='sessions'
                ORDER BY ordinal_position
            """)
            print(f"✅ Sessions table has {len(columns)} columns:")
            for col in columns:
                nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
                print(f"   - {col['column_name']}: {col['data_type']} ({nullable})")

        # Check row counts
        print("\n🔍 Checking table row counts...")
        for table in table_names:
            try:
                count = await conn.fetchval(f"SELECT COUNT(*) FROM {table}")
                print(f"   - {table}: {count} row(s)")
            except Exception as e:
                print(f"   - {table}: Error counting rows - {e}")

        await conn.close()

        print("\n" + "=" * 60)
        if missing_tables:
            print(f"⚠️  Database connection successful but {len(missing_tables)} required table(s) missing")
            print(f"   Missing: {', '.join(missing_tables)}")
            print("\n💡 To initialize the database schema, run:")
            print("   python -m api.database.init_db")
            return False
        else:
            print("✅ All checks passed! Database is properly configured.")
            return True

    except asyncpg.exceptions.InvalidPasswordError:
        print("❌ Authentication failed - invalid password")
        return False
    except asyncpg.exceptions.InvalidCatalogNameError:
        print("❌ Database does not exist")
        return False
    except asyncpg.exceptions.PostgresConnectionError as e:
        print(f"❌ Connection error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {type(e).__name__}: {e}")
        return False


async def test_connection_pool():
    """Test connection pooling."""
    print("\n" + "=" * 60)
    print("🔍 Testing Connection Pool")
    print("=" * 60)

    try:
        from api.database.connection import get_pool, close_pool

        print("\n📡 Creating connection pool...")
        pool = await get_pool()
        print(f"✅ Connection pool created")
        print(f"   Min connections: {pool._minsize}")
        print(f"   Max connections: {pool._maxsize}")

        # Test query through pool
        print("\n🔍 Testing query through pool...")
        async with pool.acquire() as conn:
            result = await conn.fetchval("SELECT 1")
            if result == 1:
                print("✅ Pool query test passed")

        # Close pool
        print("\n🔌 Closing connection pool...")
        await close_pool()
        print("✅ Connection pool closed")

        return True

    except Exception as e:
        print(f"❌ Pool test failed: {type(e).__name__}: {e}")
        return False


async def main():
    """Run all tests."""
    print("\n🚀 Starting Neon Database Verification Tests")

    # Test 1: Basic connection
    test1_passed = await test_connection()

    # Test 2: Connection pool
    test2_passed = await test_connection_pool()

    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)
    print(f"Basic Connection Test: {'✅ PASSED' if test1_passed else '❌ FAILED'}")
    print(f"Connection Pool Test: {'✅ PASSED' if test2_passed else '❌ FAILED'}")
    print("=" * 60)

    if test1_passed and test2_passed:
        print("\n🎉 All tests passed! Your Neon database is ready to use.")
        sys.exit(0)
    else:
        print("\n❌ Some tests failed. Please check the errors above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
