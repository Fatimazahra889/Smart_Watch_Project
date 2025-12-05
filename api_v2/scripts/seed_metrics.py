#!/usr/bin/env python3
"""
Seed script for generating fake metrics data with wide date ranges
to test metrics summary functionality.

This script creates realistic metrics data covering multiple months/years
with various metric types and realistic values.
"""

import sys
import asyncio
from datetime import datetime, timedelta, timezone
from typing import List, Dict, Any
import random
import uuid

# Add the app directory to the Python path
sys.path.insert(0, '/app')

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from app.models.metric import Metric
from app.models.user import User
from app.models.enums import MetricType
from app.core.database import get_db


class MetricsSeeder:
    """Class to handle metrics seeding operations."""
    
    def __init__(self, db_url: str):
        """Initialize the seeder with database connection."""
        self.engine = create_engine(db_url)
        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        
        # Define realistic value ranges for each metric type
        self.metric_ranges = {
            MetricType.SPO2: (95.0, 100.0),  # Normal SpO2 range
            MetricType.HEART_RATE: (60.0, 100.0),  # Normal resting heart rate
            MetricType.SKIN_TEMPERATURE: (32.0, 37.0),  # Normal skin temperature
            MetricType.AMBIENT_TEMPERATURE: (18.0, 25.0),  # Comfortable room temperature
            MetricType.STEPS: (0.0, 20000.0),  # Daily steps
            MetricType.SLEEP: (4.0, 10.0),  # Hours of sleep
        }
        
        # Units for each metric type
        self.metric_units = {
            MetricType.SPO2: "%",
            MetricType.HEART_RATE: "bpm",
            MetricType.SKIN_TEMPERATURE: "°C",
            MetricType.AMBIENT_TEMPERATURE: "°C",
            MetricType.STEPS: "steps",
            MetricType.SLEEP: "hours",
        }
        
        # Sensor models for each metric type
        self.sensor_models = {
            MetricType.SPO2: ["PulseOx-100", "OxySense-200", "HealthTrack-SPO2"],
            MetricType.HEART_RATE: ["HR-Monitor-1", "CardioTrack-100", "PulsePro-200"],
            MetricType.SKIN_TEMPERATURE: ["TempSense-1", "SkinTemp-100", "BodyTemp-Pro"],
            MetricType.AMBIENT_TEMPERATURE: ["AmbientTemp-1", "RoomSense-100", "EnvTemp-Pro"],
            MetricType.STEPS: ["StepCounter-1", "ActivityTrack-100", "MoveSense-Pro"],
            MetricType.SLEEP: ["SleepTrack-1", "RestMonitor-100", "SleepSense-Pro"],
        }
    
    def get_db_session(self):
        """Get database session."""
        return self.SessionLocal()
    
    def get_existing_users(self, db) -> List[str]:
        """Get list of existing user IDs from the database."""
        users = db.query(User).filter(User.deleted_at.is_(None)).all()
        return [str(user.id) for user in users]
    
    def generate_realistic_value(self, metric_type: MetricType, timestamp: datetime) -> float:
        """Generate realistic metric values based on type and time of day."""
        min_val, max_val = self.metric_ranges[metric_type]
        
        if metric_type == MetricType.HEART_RATE:
            # Heart rate varies by time of day - lower at night, higher during day
            hour = timestamp.hour
            if 2 <= hour <= 6:  # Night/sleep hours
                base_range = (50.0, 65.0)
            elif 7 <= hour <= 9:  # Morning wake-up
                base_range = (65.0, 85.0)
            elif 10 <= hour <= 18:  # Daytime activity
                base_range = (70.0, 100.0)
            else:  # Evening
                base_range = (65.0, 85.0)
            return round(random.uniform(*base_range), 1)
        
        elif metric_type == MetricType.SPO2:
            # SpO2 is generally stable but can have slight variations
            base_value = random.uniform(96.0, 99.5)
            return round(base_value, 1)
        
        elif metric_type == MetricType.STEPS:
            # Steps vary by time of day - more during active hours
            hour = timestamp.hour
            if 6 <= hour <= 9:  # Morning activity
                steps = random.randint(500, 2000)
            elif 12 <= hour <= 14:  # Lunch time
                steps = random.randint(300, 1500)
            elif 17 <= hour <= 20:  # Evening activity
                steps = random.randint(800, 2500)
            else:  # Night/low activity
                steps = random.randint(0, 500)
            return float(steps)
        
        elif metric_type == MetricType.SLEEP:
            # Sleep data - only generate during night hours
            hour = timestamp.hour
            if 22 <= hour or hour <= 6:  # Night hours (10 PM to 6 AM)
                return round(random.uniform(0.5, 2.0), 1)  # Hours of sleep in this interval
            else:
                return 0.0
        
        elif metric_type in [MetricType.SKIN_TEMPERATURE, MetricType.AMBIENT_TEMPERATURE]:
            # Temperature with slight variations
            base_value = random.uniform(min_val, max_val)
            return round(base_value, 1)
        
        else:
            # Default random value within range
            return round(random.uniform(min_val, max_val), 1)
    
    def generate_metrics_for_date_range(
        self, 
        user_id: str, 
        start_date: datetime, 
        end_date: datetime,
        metrics_per_day: int = 24  # Hourly metrics
    ) -> List[Metric]:
        """Generate metrics for a specific user over a date range."""
        metrics = []
        current_date = start_date
        
        while current_date <= end_date:
            # Generate metrics for each day
            for hour in range(metrics_per_day):
                metric_timestamp = current_date.replace(
                    hour=hour, 
                    minute=random.randint(0, 59),
                    second=random.randint(0, 59)
                )
                
                # Generate different metric types with different frequencies
                for metric_type in MetricType:
                    # Vary frequency of different metric types
                    if metric_type == MetricType.STEPS:
                        if random.random() < 0.3:  # 30% chance for steps
                            continue
                    elif metric_type == MetricType.SLEEP:
                        if random.random() < 0.7:  # 70% chance for sleep (only at night)
                            if not (22 <= hour <= 6):  # Only generate sleep data at night
                                continue
                    else:
                        if random.random() < 0.5:  # 50% chance for other metrics
                            continue
                    
                    value = self.generate_realistic_value(metric_type, metric_timestamp)
                    sensor_model = random.choice(self.sensor_models[metric_type])
                    
                    metric = Metric(
                        id=str(uuid.uuid4()),
                        metric_type=metric_type,
                        value=value,
                        unit=self.metric_units[metric_type],
                        sensor_model=sensor_model,
                        timestamp=metric_timestamp,
                        user_id=user_id
                    )
                    metrics.append(metric)
            
            current_date += timedelta(days=1)
        
        return metrics
    
    def seed_metrics(self, num_users: int = 3, months_back: int = 12):
        """Main method to seed metrics data."""
        db = self.get_db_session()
        
        try:
            # Get existing users
            user_ids = self.get_existing_users(db)
            
            if not user_ids:
                print("No users found in database. Please create users first.")
                return
            
            # Use available users (up to num_users)
            selected_users = user_ids[:min(num_users, len(user_ids))]
            print(f"Seeding metrics for {len(selected_users)} users...")
            
            total_metrics_created = 0
            
            for user_id in selected_users:
                print(f"Generating metrics for user {user_id}...")
                
                # Generate metrics for different time periods
                end_date = datetime.now(timezone.utc)
                
                # Recent data (last 7 days) - high frequency
                recent_start = end_date - timedelta(days=7)
                recent_metrics = self.generate_metrics_for_date_range(
                    user_id, recent_start, end_date, metrics_per_day=24
                )
                
                # Medium-term data (last 30 days) - medium frequency
                medium_start = end_date - timedelta(days=30)
                medium_metrics = self.generate_metrics_for_date_range(
                    user_id, medium_start, recent_start - timedelta(days=1), metrics_per_day=12
                )
                
                # Historical data (last 12 months) - low frequency
                historical_start = end_date - timedelta(days=30 * months_back)
                historical_metrics = self.generate_metrics_for_date_range(
                    user_id, historical_start, medium_start - timedelta(days=1), metrics_per_day=6
                )
                
                # Combine all metrics
                all_metrics = recent_metrics + medium_metrics + historical_metrics
                
                # Add to database in batches to avoid memory issues
                batch_size = 1000
                for i in range(0, len(all_metrics), batch_size):
                    batch = all_metrics[i:i + batch_size]
                    db.add_all(batch)
                    db.commit()
                    print(f"  Added batch {i//batch_size + 1}: {len(batch)} metrics")
                
                total_metrics_created += len(all_metrics)
                print(f"  Total metrics for user {user_id}: {len(all_metrics)}")
            
            print(f"\n✅ Successfully seeded {total_metrics_created} metrics across {len(selected_users)} users")
            print(f"📊 Date range: {months_back} months of historical data")
            print(f"📈 Metric types: {', '.join([mt.value for mt in MetricType])}")
            
        except Exception as e:
            db.rollback()
            print(f"❌ Error seeding metrics: {e}")
            raise
        finally:
            db.close()


def main():
    """Main function to run the seed script."""
    import os
    from dotenv import load_dotenv
    
    # Load environment variables
    load_dotenv()
    
    # Get database URL from environment
    database_url = os.getenv('DATABASE_URL')
    if not database_url:
        print("❌ DATABASE_URL environment variable not set")
        sys.exit(1)
    
    # Create seeder and run
    seeder = MetricsSeeder(database_url)
    
    # Configuration
    num_users = 3  # Number of users to seed metrics for
    months_back = 12  # How many months of historical data to generate
    
    print("🚀 Starting metrics seeding...")
    print(f"📋 Configuration:")
    print(f"   - Users: {num_users}")
    print(f"   - Historical data: {months_back} months")
    print(f"   - Database: {database_url.split('@')[-1] if '@' in database_url else database_url}")
    
    seeder.seed_metrics(num_users=num_users, months_back=months_back)


if __name__ == "__main__":
    main()