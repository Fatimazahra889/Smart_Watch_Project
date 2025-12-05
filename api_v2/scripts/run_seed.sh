#!/bin/bash

# Script to run metrics seeding from Docker container

echo "🔧 Setting up metrics seeding environment..."

# Check if we're in Docker container
if [ -f /.dockerenv ]; then
    echo "📦 Running inside Docker container"
    
    # Run the seed script
    cd /app
    python scripts/seed_metrics.py
    
else
    echo "🐳 Running from host - executing in Docker container"
    
    # Run inside the backend container
    docker compose exec backend python scripts/seed_metrics.py
fi

echo "✅ Seed script execution completed"