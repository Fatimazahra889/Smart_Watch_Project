from app.core.database import engine
from app.models.base import Base
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.routes import auth, metrics

app = FastAPI(
    title="IoT Backend API",
    description="A FastAPI backend for an IoT platform.",
    version="0.1.0",
)

# Add CORS middleware to allow all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.on_event("startup")
def create_db_tables():
    print("Creating database tables (if they don't exist)...")
    Base.metadata.create_all(bind=engine, checkfirst=True)
    print("Tables created successfully.")

# Include authentication router
app.include_router(auth.router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(metrics.router, prefix="/api/v1/metrics", tags=["metrics"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the IoT Backend API"}
