import uuid
from sqlalchemy import Column, String, ForeignKey, DateTime, Boolean, Enum
from sqlalchemy.orm import relationship
from app.models import Base
from sqlalchemy.sql import func
from app.models.enums import IssueSeverity

class Issue(Base):
    __tablename__ = "issues"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    issue_type = Column(String)
    description = Column(String)
    # Use SQLite-compatible Enum (native_enum=False for SQLite)
    severity = Column(Enum(IssueSeverity, native_enum=False))
    detected_at = Column(DateTime, default=func.now())
    resolved = Column(Boolean, default=False)
    user_id = Column(String(36), ForeignKey("users.id"))
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)

    reporter = relationship("User", back_populates="issues")
