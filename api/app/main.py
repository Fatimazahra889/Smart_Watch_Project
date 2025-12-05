from fastapi import FastAPI
from app.api.routes import auth, users, devices, metrics, summary, issues

app = FastAPI(
    title="IoT Backend API",
    description="A FastAPI backend for an IoT platform.",
    version="0.1.0",
)

app.include_router(auth.router, prefix="/api/v1", tags=["auth"])
app.include_router(users.router, prefix="/api/v1", tags=["users"])
app.include_router(devices.router, prefix="/api/v1", tags=["devices"])
app.include_router(metrics.router, prefix="/api/v1", tags=["metrics"])
app.include_router(summary.router, prefix="/api/v1", tags=["summary"])
app.include_router(issues.router, prefix="/api/v1", tags=["issues"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the IoT Backend API"}
