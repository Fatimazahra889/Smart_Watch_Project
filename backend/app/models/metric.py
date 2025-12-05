import uuid
from sqlalchemy import Column, String, Float, DateTime, ForeignKey, Enum
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID
from app.core.database import Base
from sqlalchemy.sql import func
from app.models.enums import MetricType

class Metric(Base):
    __tablename__ = "metrics"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    metric_type = Column(Enum(MetricType))
    value = Column(Float, nullable=True)
    unit = Column(String)
    sensor_model = Column(String)
    timestamp = Column(DateTime, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"))
    device_id = Column(UUID(as_uuid=True), ForeignKey("devices.id"), index=True)
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)

    user = relationship("User", back_populates="metrics")
    device = relationship("Device", back_populates="metrics")