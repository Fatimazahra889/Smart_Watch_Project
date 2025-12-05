import uuid
from sqlalchemy import Column, String, ForeignKey, DateTime, Boolean, Enum
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID
from app.core.database import Base
from sqlalchemy.sql import func
from app.models.enums import IssueSeverity

class Issue(Base):
    __tablename__ = "issues"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    issue_type = Column(String)
    description = Column(String)
    severity = Column(Enum(IssueSeverity))
    detected_at = Column(DateTime)
    resolved = Column(Boolean, default=False)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"))
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    deleted_at = Column(DateTime, nullable=True)

    reporter = relationship("User", back_populates="issues")
