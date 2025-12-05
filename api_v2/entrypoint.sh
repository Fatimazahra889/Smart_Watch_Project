#!/bin/sh

echo "Checking/creating database tables..."
# Run the database initialization script (it's safe to run every time now)
echo "Checking/creating database tables..."
python -m app.db_init

# Execute the main command (passed from docker-compose)
exec "$@"
