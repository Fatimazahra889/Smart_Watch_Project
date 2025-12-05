from fastapi import APIRouter, Depends, HTTPException, Body
from sqlalchemy.orm import Session
from app.api import deps
from app.schemas.user import User as UserSchema, UserCreate, UserUpdate, UserVerifyEmail, ResetPasswordWithCodeRequest
from app.services.user_service import user_service
from app.models.user import User
from pydantic import BaseModel, EmailStr
from uuid import UUID

router = APIRouter()

class ForgotPasswordRequest(BaseModel):
    email: EmailStr

@router.post("/users/", response_model=UserSchema)
def create_user(
    *,
    db: Session = Depends(deps.get_db),
    user_in: UserCreate
):
    """
    Create new user.
    """
    user = user_service.get_by_email(db, email=user_in.email)
    if user:
        raise HTTPException(
            status_code=400,
            detail="The user with this username already exists in the system.",
        )
    user = user_service.create_user(db, user_in=user_in)
    return user

@router.get("/users/", response_model=list[UserSchema])
def read_users(
    db: Session = Depends(deps.get_db),
    skip: int = 0,
    limit: int = 100,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Retrieve users.
    """
    if not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this resource."
        )
    users = user_service.get_multi(db, skip=skip, limit=limit)
    return users

@router.get("/users/{user_id}", response_model=UserSchema)
def read_user_by_id(
    user_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_user),
):
    """
    Get a specific user by ID.
    """
    if str(user_id) != str(current_user.id) and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this resource."
        )
    user = user_service.get(db, id=str(user_id))
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user

@router.put("/users/{user_id}", response_model=UserSchema)
def update_user(
    *,
    db: Session = Depends(deps.get_db),
    user_id: UUID,
    user_in: UserUpdate,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Update a user.
    """
    if str(user_id) != str(current_user.id) and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to perform this action."
        )
    user = user_service.get(db, id=str(user_id))
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    user = user_service.update_user(db, db_obj=user, obj_in=user_in)
    return user

@router.delete("/users/{user_id}", response_model=UserSchema)
def delete_user(
    *,
    db: Session = Depends(deps.get_db),
    user_id: UUID,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Delete a user (soft delete).
    """
    if str(user_id) != str(current_user.id) and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to perform this action."
        )
    user = user_service.delete_user(db, id=str(user_id))
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user

@router.post("/users/verify-email", response_model=UserSchema)
def verify_email(
    *,
    db: Session = Depends(deps.get_db),
    request: UserVerifyEmail
):
    """
    Verify user email with a 6-digit code.
    """
    user = user_service.verify_email_with_code(db, email=request.email, code=request.code)
    return user

@router.post("/users/forgot-password")
def forgot_password(
    *,
    db: Session = Depends(deps.get_db),
    request: ForgotPasswordRequest
):
    """
    Send password reset code.
    """
    user_service.initiate_password_reset(db, email=request.email)
    return {"message": "If a user with that email exists, a password reset code has been sent."}

@router.post("/users/reset-password", response_model=UserSchema)
def reset_password(
    *,
    db: Session = Depends(deps.get_db),
    request: ResetPasswordWithCodeRequest
):
    """
    Reset user password with a 6-digit code.
    """
    user = user_service.reset_password_with_code(
        db, email=request.email, code=request.code, new_password=request.new_password
    )
    return user