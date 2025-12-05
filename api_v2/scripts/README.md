# Metrics Seed Script

This script generates realistic fake metrics data for testing the metrics summary functionality in the IoT Bracelet Connecté application.

## Overview

The seed script creates comprehensive metrics data covering:
- **Multiple users** (uses existing users from database)
- **Wide date ranges** (up to 12 months of historical data)
- **All metric types** (SPO2, Heart Rate, Temperature, Steps, Sleep)
- **Realistic values** with time-of-day variations
- **Different sampling frequencies** (hourly, daily, monthly)

## Generated Data Patterns

### Date Ranges
- **Recent data**: Last 7 days with hourly frequency (24 metrics/day)
- **Medium-term**: Last 30 days with 12-hour frequency (12 metrics/day)  
- **Historical**: Up to 12 months with 4-hour frequency (6 metrics/day)

### Metric Types & Realistic Values

| Metric Type | Normal Range | Unit | Description |
|-------------|--------------|------|-------------|
| SPO2 | 95-100% | % | Blood oxygen saturation |
| Heart Rate | 60-100 bpm | bpm | Varies by time of day |
| Skin Temperature | 32-37°C | °C | Body surface temperature |
| Ambient Temperature | 18-25°C | °C | Room/environment temperature |
| Steps | 0-20,000 | steps | Activity tracking |
| Sleep | 4-10 hours | hours | Sleep duration (night only) |

### Time-of-Day Variations
- **Heart Rate**: Lower at night (50-65 bpm), higher during day (70-100 bpm)
- **Steps**: Peak during morning/evening, minimal at night
- **Sleep**: Only generated during night hours (10 PM - 6 AM)

## Usage

### Prerequisites
- Backend service running with database
- Existing users in the database
- Environment variables configured (DATABASE_URL)

### Running the Script

#### Option 1: Using Make (Recommended)
```bash
# Run from host - automatically detects Docker environment
make seed-metrics

# Run directly in Docker container
make seed-metrics-docker
```

#### Option 2: Direct Execution
```bash
# From host machine
docker compose exec backend python scripts/seed_metrics.py

# Inside Docker container
python scripts/seed_metrics.py
```

### Configuration

Edit `scripts/seed_metrics.py` to modify:
- `num_users`: Number of users to seed metrics for (default: 3)
- `months_back`: Months of historical data (default: 12)

## Testing Summary Functionality

After running the seed script, you can test the metrics summary endpoints:

### Daily Summary
```bash
# Get daily metrics summary for all types
curl -H "Authorization: Bearer <token>" \
  "http://localhost:8000/api/metrics/summary"

# Get summary for specific metric type and date range
curl -H "Authorization: Bearer <token>" \
  "http://localhost:8000/api/metrics/summary?metric_type=heart_rate&start_date=2024-01-01&end_date=2024-12-31"
```

### Health Prediction
```bash
# Get health prediction based on metrics
curl -H "Authorization: Bearer <token>" \
  "http://localhost:8000/api/metrics/health-prediction"
```

## Data Volume

With default settings (3 users, 12 months):
- **Recent data**: ~500 metrics per user
- **Medium-term**: ~350 metrics per user  
- **Historical**: ~2,000 metrics per user
- **Total**: ~8,500 metrics across all users

## Notes

- The script uses existing users from the database
- Metrics are generated with realistic patterns and variations
- Data includes gaps and missing values to simulate real-world scenarios
- All timestamps are in UTC timezone
- Sensor models vary randomly for authenticity

## Troubleshooting

**No users found**: Create users first using the auth endpoints
**Database connection issues**: Check DATABASE_URL environment variable
**Permission errors**: Ensure script has execute permissions