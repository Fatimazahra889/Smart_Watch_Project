from pydantic import BaseModel, ConfigDict
from typing import Optional
import uuid
from datetime import datetime
from app.models.enums import IssueSeverity

# Shared properties
class IssueBase(BaseModel):
    issue_type: Optional[str] = None
    description: Optional[str] = None
    severity: Optional[IssueSeverity] = None
    detected_at: Optional[datetime] = None
    resolved: Optional[bool] = False

# Properties to receive via API on creation
class IssueCreate(IssueBase):
    issue_type: str
    description: str
    severity: IssueSeverity
    detected_at: datetime

# Properties to receive via API on update
class IssueUpdate(IssueBase):
    pass

# Properties shared by models in DB
class IssueInDBBase(IssueBase):
    id: uuid.UUID
    user_id: uuid.UUID
    created_at: datetime
    updated_at: datetime
    deleted_at: Optional[datetime] = None

    model_config = ConfigDict(from_attributes=True)

# Properties to return to client
class Issue(IssueInDBBase):
    pass
