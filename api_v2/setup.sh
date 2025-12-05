#!/bin/bash
set -e

# Export the password as an environment variable so pg_isready can see it automatically.
# We use single quotes '' to ensure the & symbol doesn't break Bash.
export PGPASSWORD='Abdelhamid&62625'

echo "Starting IoT Backend setup..."

# Wait for database to be ready
# Note: We use -d for the database NAME (postgres), not the password.
echo "Waiting for PostgreSQL to be ready..."
until pg_isready -h braceletbd.postgres.database.azure.com -p 5432 -U "admin_iotdb" -d "postgres"; do
  echo "PostgreSQL is unavailable - sleeping 5s"
  sleep 5
done

echo "PostgreSQL is up and running!"

# Run database migrations
echo "Running database migrations..."
# Ideally, rely on the DATABASE_URL env var set in Azure, but if you must hardcode:
# export DATABASE_URL="postgresql://admin_iotdb:Abdelhamid%2662625@braceletbd.postgres.database.azure.com:5432/postgres"
uv run alembic upgrade head

echo "Database migrations completed successfully!"

# Start the application
echo "Starting FastAPI application..."
# Ensure we bind to 0.0.0.0 for Azure
exec uvicorn main:app --host 0.0.0.0 --port 8000