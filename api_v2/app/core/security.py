from typing import Optional
import uuid
from datetime import datetime, timedelta, timezone
from typing import Any, Union
from jose import jwt
from passlib.context import CryptContext
from app.config.settings import settings
from itsdangerous import URLSafeTimedSerializer
import secrets
import random

from app.services.blacklist_service import blacklist_service


pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

ALGORITHM = settings.ALGORITHM

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def create_access_token(
    subject: Union[str, Any], expires_delta: Optional[timedelta] = None
) -> str:
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(
            minutes=43200  # Default to 30 days
        )
    
    # Generate a unique JWT ID (JTI) for token blacklisting
    jti = str(uuid.uuid4())
    
    to_encode = {
        "exp": expire, 
        "sub": str(subject),
        "jti": jti
    }
    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=ALGORITHM)
    
    # remove from blacklist (in case it was previously blacklisted)
    blacklist_service.remove_from_blacklist(jti)
    
    return encoded_jwt

def generate_api_key() -> str:
    return secrets.token_urlsafe(32)

def generate_6_digit_code() -> str:
    """Generates a random 6-digit code."""
    return str(random.randint(100000, 999999))

def generate_verification_token(email: str) -> str:
    serializer = URLSafeTimedSerializer(settings.SECRET_KEY)
    return serializer.dumps(email, salt='email-verification')

def verify_verification_token(token: str) -> str | None:
    serializer = URLSafeTimedSerializer(settings.SECRET_KEY)
    try:
        email = serializer.loads(
            token,
            salt='email-verification',
            max_age=3600  # 1 hour
        )
        return email
    except Exception:
        return None

def generate_password_reset_token(email: str) -> str:
    serializer = URLSafeTimedSerializer(settings.SECRET_KEY)
    return serializer.dumps(email, salt='password-reset')

def verify_password_reset_token(token: str) -> str | None:
    serializer = URLSafeTimedSerializer(settings.SECRET_KEY)
    try:
        email = serializer.loads(
            token,
            salt='password-reset',
            max_age=3600  # 1 hour
        )
        return email
    except Exception:
        return None
