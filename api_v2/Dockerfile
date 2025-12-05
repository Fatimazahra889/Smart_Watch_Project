# Use a base image for Python and slim footprint
FROM python:3.12-slim

# Install system dependencies including postgresql-client for pg_isready
RUN apt-get update && apt-get install -y \
    postgresql-client \
    && rm -rf /var/lib/apt/lists/*

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

# Set the Python path
ENV PYTHONPATH=/app

# First copy only project metadata (for better Docker caching)
COPY pyproject.toml uv.lock* ./

# Install dependencies using uv (no cache, frozen lockfile)
RUN uv sync --frozen --no-editable

# Now copy the rest of the app
COPY . .

# Expose port
EXPOSE 8000

# Copy and make setup script executable
COPY setup.sh /app/setup.sh
RUN chmod +x /app/setup.sh

# Use setup script as entrypoint
CMD ["/app/setup.sh", "uv", "run", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]