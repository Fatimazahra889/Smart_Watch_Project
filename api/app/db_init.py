from app.core.database import Base, engine
from app.models.user import User
from app.models.device import Device
from app.models.metric import Metric

print("Creating database tables (if they don't exist)...")
Base.metadata.create_all(bind=engine, checkfirst=True)
print("Tables created successfully.")