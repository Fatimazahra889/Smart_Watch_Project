from __future__ import annotations
from pydantic import BaseModel, EmailStr, ConfigDict
from typing import Optional, List
import uuid
from datetime import datetime

# Shared properties
class UserBase(BaseModel):
    email: Optional[EmailStr] = None
    name: Optional[str] = None
    is_admin: Optional[bool] = False

# Properties to receive via API on creation
class UserCreate(UserBase):
    email: EmailStr
    password: str
    name: str

# Properties to receive via API on update
class UserUpdate(UserBase):
    password: Optional[str] = None

class UserVerifyEmail(BaseModel):
    email: EmailStr
    code: str

class ResetPasswordWithCodeRequest(BaseModel):
    email: EmailStr
    code: str
    new_password: str

# Properties shared by models in DB
class UserInDBBase(UserBase):
    id: uuid.UUID
    created_at: datetime
    updated_at: datetime
    email_verified_at: Optional[datetime] = None
    deleted_at: Optional[datetime] = None

    model_config = ConfigDict(from_attributes=True)

# Properties to return to client
class User(UserInDBBase):
    devices: List["Device"] = []
    issues: List["Issue"] = []
    metrics: List["Metric"] = []

# Properties stored in DB
class UserInDB(UserInDBBase):
    hashed_password: str