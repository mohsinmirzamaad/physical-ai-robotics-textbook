"""
FastAPI application entry point
Handles CORS, error handlers, and route registration
"""

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

from api.database.connection import init_database, close_pool

load_dotenv()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events
    """
    # Startup
    print("Starting up Physical AI Textbook API...")
    await init_database()
    print("Database initialized")

    yield

    # Shutdown
    print("Shutting down...")
    await close_pool()
    print("Database connections closed")


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for RAG chatbot, authentication, and content personalization",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        os.getenv("AUTH_URL", "http://localhost:3000"),
        "http://localhost:3000",
        "https://*.vercel.app",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Error handlers
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Handle validation errors"""
    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content={
            "error": "Validation Error",
            "detail": exc.errors(),
            "body": exc.body
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle general exceptions"""
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "error": "Internal Server Error",
            "detail": str(exc)
        }
    )


# Health check endpoint
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Physical AI Textbook API",
        "version": "1.0.0"
    }


# Root endpoint
@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "docs": "/docs",
        "health": "/health"
    }


# Import and register routes (will be created in subsequent tasks)
# from api.routes import auth, chat, content
# app.include_router(auth.router, prefix="/api/auth", tags=["Authentication"])
# app.include_router(chat.router, prefix="/api/chat", tags=["Chatbot"])
# app.include_router(content.router, prefix="/api/content", tags=["Content"])


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
