import uuid
from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from app.models import Base

class User(Base):
    __tablename__ = "users"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String)
    email = Column(String, unique=True, index=True, nullable=False)
    email_verified_at = Column(DateTime, nullable=True)
    hashed_password = Column(String, nullable=False)
    is_admin = Column(Boolean, default=False)
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)
    verification_code = Column(String, nullable=True)
    verification_code_expires_at = Column(DateTime, nullable=True)
    password_reset_code = Column(String, nullable=True)
    password_reset_code_expires_at = Column(DateTime, nullable=True)

    devices = relationship("Device", back_populates="owner")
    issues = relationship("Issue", back_populates="reporter")
    metrics = relationship("Metric", back_populates="user")