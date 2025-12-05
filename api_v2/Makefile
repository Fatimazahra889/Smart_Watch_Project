run:
	uv run uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

seed-metrics:
	chmod +x scripts/run_seed.sh && ./scripts/run_seed.sh

seed-metrics-docker:
	docker compose exec backend python scripts/seed_metrics.py