from datetime import datetime, timedelta, timezone
from typing import Optional
from sqlalchemy.orm import Session
from app.models.user import User
from app.core.exceptions import UserNotFoundException, UserAlreadyExistsException
from app.core.security import get_password_hash, generate_6_digit_code


class UserRepository:
    """Repository for user database operations."""
    
    def __init__(self, db: Session):
        self.db = db

    def get_user_by_id(self, user_id: str) -> Optional[User]:
        """Get user by ID."""
        return self.db.query(User).filter(
            User.id == user_id,
            User.deleted_at.is_(None)
        ).first()

    def get_user_by_email(self, email: str) -> Optional[User]:
        """Get user by email."""
        return self.db.query(User).filter(
            User.email == email,
            User.deleted_at.is_(None)
        ).first()

    def create_user(self, name: str, email: str, password: str) -> User:
        """Create a new user with email verification."""
        # Check if user already exists
        existing_user = self.get_user_by_email(email)
        if existing_user:
            raise UserAlreadyExistsException(email)

        # Generate verification code
        verification_code = generate_6_digit_code()
        verification_expires = datetime.now(timezone.utc) + timedelta(hours=24)  # 24 hours expiry

        # Create user
        user = User(
            name=name,
            email=email,
            hashed_password=get_password_hash(password),
            verification_code=verification_code,
            verification_code_expires_at=verification_expires
        )
        
        self.db.add(user)
        self.db.commit()
        self.db.refresh(user)
        return user

    def update_user(self, user_id: str, **kwargs) -> User:
        """Update user fields."""
        user = self.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)

        for key, value in kwargs.items():
            if hasattr(user, key):
                setattr(user, key, value)

        user.updated_at = datetime.now(timezone.utc)
        self.db.commit()
        self.db.refresh(user)
        return user

    def delete_user(self, user_id: str) -> bool:
        """Soft delete user by setting deleted_at timestamp."""
        user = self.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)

        user.deleted_at = datetime.now(timezone.utc)
        self.db.commit()
        return True

    def verify_email(self, email: str, verification_code: str) -> bool:
        """Verify user email with verification code."""
        user = self.get_user_by_email(email)
        if not user:
            raise UserNotFoundException(f"email: {email}")

        # Check if code matches and hasn't expired
        current_time = datetime.now(timezone.utc)
        if (user.verification_code == verification_code and
            user.verification_code_expires_at):
            
            # Handle both naive and aware datetimes
            expires_at = user.verification_code_expires_at
            if expires_at.tzinfo is None:
                # Naive datetime - assume UTC
                expires_at = expires_at.replace(tzinfo=timezone.utc)
            
            if expires_at > current_time:
                user.email_verified_at = current_time
                user.verification_code = None
                user.verification_code_expires_at = None
                self.db.commit()
                return True
        
        return False

    def resend_verification_code(self, email: str) -> str:
        """Generate and set new verification code for user."""
        user = self.get_user_by_email(email)
        if not user:
            raise UserNotFoundException(f"email: {email}")

        # Generate new verification code
        verification_code = generate_6_digit_code()
        verification_expires = datetime.now(timezone.utc) + timedelta(hours=24)

        user.verification_code = verification_code
        user.verification_code_expires_at = verification_expires
        user.updated_at = datetime.now(timezone.utc)
        
        self.db.commit()
        return verification_code

    def set_password_reset_code(self, email: str) -> str:
        """Set password reset code for user."""
        user = self.get_user_by_email(email)
        if not user:
            raise UserNotFoundException(f"email: {email}")

        reset_code = generate_6_digit_code()
        reset_expires = datetime.now(timezone.utc) + timedelta(hours=1)  # 1 hour expiry

        user.password_reset_code = reset_code
        user.password_reset_code_expires_at = reset_expires
        user.updated_at = datetime.now(timezone.utc)
        
        self.db.commit()
        return reset_code

    def reset_password(self, email: str, reset_code: str, new_password: str) -> bool:
        """Reset user password using reset code."""
        user = self.get_user_by_email(email)
        if not user:
            raise UserNotFoundException(f"email: {email}")

        # Check if reset code matches and hasn't expired
        current_time = datetime.now(timezone.utc)
        if (user.password_reset_code == reset_code and
            user.password_reset_code_expires_at):
            
            # Handle both naive and aware datetimes
            expires_at = user.password_reset_code_expires_at
            if expires_at.tzinfo is None:
                # Naive datetime - assume UTC
                expires_at = expires_at.replace(tzinfo=timezone.utc)
            
            if expires_at > current_time:
                user.hashed_password = get_password_hash(new_password)
                user.password_reset_code = None
                user.password_reset_code_expires_at = None
                user.updated_at = current_time
                self.db.commit()
                return True
        
        return False

    def change_password(self, user_id: str, new_password: str) -> bool:
        """Change user password directly."""
        user = self.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)

        user.hashed_password = get_password_hash(new_password)
        user.updated_at = datetime.now(timezone.utc)
        self.db.commit()
        return True

    def get_all_users(self, skip: int = 0, limit: int = 100) -> list[User]:
        """Get all users with pagination."""
        return self.db.query(User).filter(
            User.deleted_at.is_(None)
        ).offset(skip).limit(limit).all()

    def get_users_count(self) -> int:
        """Get total count of active users."""
        return self.db.query(User).filter(
            User.deleted_at.is_(None)
        ).count()