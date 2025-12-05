# Use a base image for Python and slim footprint
FROM python:3.13-slim

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

# migrate DB and start the application
# Copy and make entrypoint executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# migrate DB and start the application
CMD ["/entrypoint.sh", "uv", "run", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]