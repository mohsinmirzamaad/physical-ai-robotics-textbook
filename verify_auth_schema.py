"""Verify Better-Auth schema compatibility"""
import asyncio
import os
import sys
from dotenv import load_dotenv
import asyncpg

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

load_dotenv()

async def verify_schema():
    print("=" * 60)
    print("Verifying Better-Auth Schema Compatibility")
    print("=" * 60)

    db_url = os.getenv("DATABASE_URL")
    conn = await asyncpg.connect(db_url)

    # Check users table
    print("\n[USERS TABLE]")
    users_cols = await conn.fetch("""
        SELECT column_name, data_type, is_nullable, column_default
        FROM information_schema.columns
        WHERE table_name='users'
        ORDER BY ordinal_position
    """)

    required_users_cols = {
        'id': 'uuid',
        'email': 'character varying',
        'name': 'character varying',
        'email_verified': 'boolean',
        'created_at': 'timestamp with time zone',
        'updated_at': 'timestamp with time zone'
    }

    users_schema = {c['column_name']: c['data_type'] for c in users_cols}

    for col, dtype in required_users_cols.items():
        if col in users_schema:
            if users_schema[col] == dtype:
                print(f"  OK: {col} ({dtype})")
            else:
                print(f"  WARNING: {col} type mismatch - expected {dtype}, got {users_schema[col]}")
        else:
            print(f"  MISSING: {col} ({dtype})")

    # Check sessions table
    print("\n[SESSIONS TABLE]")
    sessions_cols = await conn.fetch("""
        SELECT column_name, data_type, is_nullable, column_default
        FROM information_schema.columns
        WHERE table_name='sessions'
        ORDER BY ordinal_position
    """)

    print(f"Found {len(sessions_cols)} columns:")
    for c in sessions_cols:
        nullable = "NULL" if c['is_nullable'] == 'YES' else "NOT NULL"
        default = f" DEFAULT {c['column_default']}" if c['column_default'] else ""
        print(f"  - {c['column_name']}: {c['data_type']} ({nullable}){default}")

    required_sessions_cols = {
        'id': 'uuid',
        'user_id': 'uuid',
        'expires_at': 'timestamp with time zone',
        'token': 'text',
        'created_at': 'timestamp with time zone'
    }

    sessions_schema = {c['column_name']: c['data_type'] for c in sessions_cols}

    print("\nValidating required columns:")
    for col, dtype in required_sessions_cols.items():
        if col in sessions_schema:
            if sessions_schema[col] == dtype:
                print(f"  OK: {col} ({dtype})")
            else:
                print(f"  WARNING: {col} type mismatch - expected {dtype}, got {sessions_schema[col]}")
        else:
            print(f"  MISSING: {col} ({dtype})")

    # Check user_preferences table
    print("\n[USER_PREFERENCES TABLE]")
    prefs_cols = await conn.fetch("""
        SELECT column_name, data_type, is_nullable
        FROM information_schema.columns
        WHERE table_name='user_preferences'
        ORDER BY ordinal_position
    """)

    for c in prefs_cols:
        nullable = "NULL" if c['is_nullable'] == 'YES' else "NOT NULL"
        print(f"  - {c['column_name']}: {c['data_type']} ({nullable})")

    # Check indexes
    print("\n[INDEXES]")
    indexes = await conn.fetch("""
        SELECT indexname, tablename, indexdef
        FROM pg_indexes
        WHERE schemaname='public'
        ORDER BY tablename, indexname
    """)

    for idx in indexes:
        print(f"  - {idx['tablename']}.{idx['indexname']}")

    # Check foreign keys
    print("\n[FOREIGN KEYS]")
    fkeys = await conn.fetch("""
        SELECT
            tc.table_name,
            kcu.column_name,
            ccu.table_name AS foreign_table_name,
            ccu.column_name AS foreign_column_name
        FROM information_schema.table_constraints AS tc
        JOIN information_schema.key_column_usage AS kcu
            ON tc.constraint_name = kcu.constraint_name
        JOIN information_schema.constraint_column_usage AS ccu
            ON ccu.constraint_name = tc.constraint_name
        WHERE tc.constraint_type = 'FOREIGN KEY'
        ORDER BY tc.table_name
    """)

    for fk in fkeys:
        print(f"  - {fk['table_name']}.{fk['column_name']} -> {fk['foreign_table_name']}.{fk['foreign_column_name']}")

    await conn.close()

    print("\n" + "=" * 60)
    print("Schema verification complete!")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(verify_schema())
