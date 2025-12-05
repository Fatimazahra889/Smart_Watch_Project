from typing import Generator
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, APIKeyHeader
from jose import jwt
from pydantic import ValidationError
from sqlalchemy.orm import Session
from app.core.database import SessionLocal
from app.schemas.token import TokenPayload
from app.models.user import User
from app.models.device import Device
from app.services.user_service import user_service
from app.repositories.device_repository import device_repository
from app.config.settings import settings

reusable_oauth2 = OAuth2PasswordBearer(
    tokenUrl="/api/v1/token"
)
api_key_scheme = APIKeyHeader(name="X-API-KEY")

def get_db() -> Generator:
    try:
        db = SessionLocal()
        yield db
    finally:
        db.close()

def get_current_user(
    db: Session = Depends(get_db), token: str = Depends(reusable_oauth2)
) -> User:
    try:
        payload = jwt.decode(
            token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM]
        )
        token_data = TokenPayload(**payload)
    except (jwt.JWTError, ValidationError):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Could not validate credentials",
        )
    user = user_service.get(db, id=token_data.sub)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user

def get_current_active_user(
    current_user: User = Depends(get_current_user),
) -> User:
    # if not crud.user.is_active(current_user):
    #     raise HTTPException(status_code=400, detail="Inactive user")
    return current_user

def get_current_device(
    api_key: str = Depends(api_key_scheme), db: Session = Depends(get_db)
) -> Device:
    device = device_repository.get_by_api_key(db, api_key=api_key)
    if not device:
        raise HTTPException(status_code=403, detail="Could not validate credentials")
    return device
