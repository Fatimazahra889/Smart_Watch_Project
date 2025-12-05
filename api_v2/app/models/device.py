import uuid
from sqlalchemy import Column, String, ForeignKey, DateTime, Boolean
from sqlalchemy.orm import relationship
from app.models import Base
from sqlalchemy.sql import func

class Device(Base):
    __tablename__ = "devices"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String)
    serial_number = Column(String, unique=True, index=True)
    api_key = Column(String, unique=True, index=True)
    model = Column(String)
    firmware_version = Column(String)
    is_active = Column(Boolean, default=True)
    registered_at = Column(DateTime, default=func.now())
    user_id = Column(String(36), ForeignKey("users.id"))
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)

    owner = relationship("User", back_populates="devices")