"""
Database connection module for Neon Postgres.
Provides connection pooling and async database operations.
"""

import os
import asyncpg
from typing import Optional
from dotenv import load_dotenv

load_dotenv()

# Database connection pool
_pool: Optional[asyncpg.Pool] = None


async def get_pool() -> asyncpg.Pool:
    """
    Get or create the database connection pool.

    Returns:
        asyncpg.Pool: Database connection pool
    """
    global _pool

    if _pool is None:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise ValueError("DATABASE_URL environment variable is not set")

        _pool = await asyncpg.create_pool(
            database_url,
            min_size=2,
            max_size=10,
            command_timeout=60,
            max_queries=50000,
            max_inactive_connection_lifetime=300
        )

    return _pool


async def close_pool():
    """Close the database connection pool."""
    global _pool
    if _pool is not None:
        await _pool.close()
        _pool = None


async def execute_query(query: str, *args):
    """
    Execute a query that doesn't return results (INSERT, UPDATE, DELETE).

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Status message from the database
    """
    pool = await get_pool()
    async with pool.acquire() as connection:
        return await connection.execute(query, *args)


async def fetch_one(query: str, *args):
    """
    Fetch a single row from the database.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Single row as a Record object or None
    """
    pool = await get_pool()
    async with pool.acquire() as connection:
        return await connection.fetchrow(query, *args)


async def fetch_all(query: str, *args):
    """
    Fetch all rows from the database.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        List of Record objects
    """
    pool = await get_pool()
    async with pool.acquire() as connection:
        return await connection.fetch(query, *args)


async def fetch_val(query: str, *args):
    """
    Fetch a single value from the database.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Single value
    """
    pool = await get_pool()
    async with pool.acquire() as connection:
        return await connection.fetchval(query, *args)


async def init_database():
    """
    Initialize the database by running the schema.sql file.
    Should be called on application startup.
    """
    pool = await get_pool()

    # Read schema file
    schema_path = os.path.join(os.path.dirname(__file__), "schema.sql")
    with open(schema_path, "r") as f:
        schema_sql = f.read()

    # Execute schema
    async with pool.acquire() as connection:
        await connection.execute(schema_sql)

    print("Database schema initialized successfully")
