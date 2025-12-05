import uuid
from sqlalchemy import Column, String, Float, DateTime, ForeignKey, Enum
from sqlalchemy.orm import relationship
from app.models import Base
from sqlalchemy.sql import func
from app.models.enums import MetricType

class Metric(Base):
    __tablename__ = "metrics"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    metric_type = Column(Enum(MetricType, native_enum=False))
    value = Column(Float, nullable=True)
    unit = Column(String)
    sensor_model = Column(String)
    timestamp = Column(DateTime, index=True, default=func.now())
    user_id = Column(String(36), ForeignKey("users.id"))
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)

    user = relationship("User", back_populates="metrics")