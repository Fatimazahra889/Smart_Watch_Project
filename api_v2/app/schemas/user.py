from pydantic import BaseModel, EmailStr, Field, validator
from typing import Optional
from datetime import datetime
import re


class UserCreate(BaseModel):
    """Schema for user registration."""
    name: str = Field(..., min_length=1, max_length=100, description="User's full name")
    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., min_length=8, max_length=100, description="User's password")

    @validator('password')
    def validate_password_strength(cls, v):
        """Validate password strength."""
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not re.search(r'\d', v):
            raise ValueError('Password must contain at least one digit')
        if not re.search(r'[!@#$%^&*(),.?":{}|<>]', v):
            raise ValueError('Password must contain at least one special character')
        return v


class UserLogin(BaseModel):
    """Schema for user authentication."""
    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., description="User's password")


class UserResponse(BaseModel):
    """Schema for API responses (excluding sensitive fields)."""
    id: str
    name: str
    email: str
    email_verified_at: Optional[datetime] = None
    is_admin: bool = False
    created_at: datetime
    updated_at: datetime
    deleted_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserUpdate(BaseModel):
    """Schema for profile updates."""
    name: Optional[str] = Field(None, min_length=1, max_length=100, description="User's full name")
    email: Optional[EmailStr] = Field(None, description="User's email address")

    @validator('name')
    def validate_name(cls, v):
        if v is not None and len(v.strip()) == 0:
            raise ValueError('Name cannot be empty or whitespace only')
        return v


class EmailVerification(BaseModel):
    """Schema for email verification requests."""
    email: EmailStr = Field(..., description="Email to verify")
    verification_code: str = Field(..., min_length=6, max_length=6, description="6-digit verification code")


class ResendCode(BaseModel):
    """Schema for resending verification codes."""
    email: EmailStr = Field(..., description="Email to resend verification code to")


class UserInDB(UserResponse):
    """Schema for user data in database (includes hashed password)."""
    hashed_password: str
    verification_code: Optional[str] = None
    verification_code_expires_at: Optional[datetime] = None
    password_reset_code: Optional[str] = None
    password_reset_code_expires_at: Optional[datetime] = None