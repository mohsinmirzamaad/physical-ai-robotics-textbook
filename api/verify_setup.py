"""
Setup verification script
Run this to check if all environment variables are configured correctly
"""

import os
import sys
from pathlib import Path

def check_env_file():
    """Check if .env file exists"""
    env_path = Path(".env")
    if not env_path.exists():
        print("❌ .env file not found!")
        print("   Run: cp .env.example .env")
        return False
    print("✅ .env file exists")
    return True

def check_env_vars():
    """Check if required environment variables are set"""
    required_vars = [
        "OPENAI_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "DATABASE_URL",
        "AUTH_SECRET"
    ]

    missing = []
    placeholder = []

    for var in required_vars:
        value = os.getenv(var)
        if not value:
            missing.append(var)
            print(f"❌ {var} is not set")
        elif "your-" in value or "sk-your" in value or "generate-" in value:
            placeholder.append(var)
            print(f"⚠️  {var} still has placeholder value")
        else:
            print(f"✅ {var} is set")

    return len(missing) == 0 and len(placeholder) == 0

def check_openai_key():
    """Verify OpenAI API key format"""
    key = os.getenv("OPENAI_API_KEY")
    if not key:
        return False

    if not key.startswith("sk-"):
        print("❌ OpenAI API key should start with 'sk-'")
        return False

    print("✅ OpenAI API key format looks correct")
    return True

def check_qdrant_url():
    """Verify Qdrant URL format"""
    url = os.getenv("QDRANT_URL")
    if not url:
        return False

    if not url.startswith("https://"):
        print("❌ Qdrant URL should start with 'https://'")
        return False

    if ".qdrant.io" not in url:
        print("⚠️  Qdrant URL should contain '.qdrant.io'")

    print("✅ Qdrant URL format looks correct")
    return True

def check_database_url():
    """Verify database URL format"""
    url = os.getenv("DATABASE_URL")
    if not url:
        return False

    if not url.startswith("postgresql://"):
        print("❌ Database URL should start with 'postgresql://'")
        return False

    if "sslmode=require" not in url:
        print("⚠️  Database URL should include 'sslmode=require'")

    print("✅ Database URL format looks correct")
    return True

def check_docs_directory():
    """Check if docs directory exists with markdown files"""
    docs_path = Path("docs")
    if not docs_path.exists():
        print("❌ docs/ directory not found")
        return False

    md_files = list(docs_path.rglob("*.md"))
    if len(md_files) == 0:
        print("❌ No markdown files found in docs/")
        return False

    print(f"✅ Found {len(md_files)} markdown files in docs/")
    return True

def main():
    print("=" * 60)
    print("🔍 Environment Setup Verification")
    print("=" * 60)
    print()

    # Load .env file
    try:
        from dotenv import load_dotenv
        load_dotenv()
        print("✅ Loaded .env file")
    except ImportError:
        print("⚠️  python-dotenv not installed, run: pip install python-dotenv")

    print()
    print("Checking configuration...")
    print("-" * 60)

    checks = [
        ("Environment file", check_env_file),
        ("Environment variables", check_env_vars),
        ("OpenAI API key", check_openai_key),
        ("Qdrant URL", check_qdrant_url),
        ("Database URL", check_database_url),
        ("Documentation files", check_docs_directory),
    ]

    results = []
    for name, check_func in checks:
        print(f"\n{name}:")
        try:
            result = check_func()
            results.append(result)
        except Exception as e:
            print(f"❌ Error: {e}")
            results.append(False)

    print()
    print("=" * 60)

    if all(results):
        print("✅ All checks passed! You're ready to run the embedding script.")
        print()
        print("Next steps:")
        print("1. Run: python -m scripts.embed_content")
        print("2. Start backend: uvicorn main:app --reload")
        print("3. Start frontend: npm start")
    else:
        print("❌ Some checks failed. Please fix the issues above.")
        print()
        print("See QUICK_SETUP.md for detailed instructions.")
        sys.exit(1)

if __name__ == "__main__":
    main()
