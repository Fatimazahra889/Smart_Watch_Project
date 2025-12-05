from pydantic import BaseModel
from typing import Optional


class Token(BaseModel):
    """Token schema for login response."""
    access_token: str
    token_type: str = "bearer"


class TokenPayload(BaseModel):
    """JWT payload structure."""
    sub: Optional[str] = None
    exp: Optional[int] = None
    jti: Optional[str] = None


class TokenData(BaseModel):
    """Token validation data."""
    user_id: Optional[str] = None
    email: Optional[str] = None