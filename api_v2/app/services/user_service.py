from datetime import datetime, timedelta, timezone
from typing import Optional, Tuple, Any
from sqlalchemy.orm import Session
from app.models.user import User
from app.repositories.user_repository import UserRepository
from app.services.mail_service import mail_service
from app.core.security import (
    verify_password,
    create_access_token,
    generate_6_digit_code,
    get_password_hash
)
from app.core.exceptions import (
    UserNotFoundException,
    InvalidCredentialsException,
    InvalidEmailException,
    UserAlreadyExistsException,
    EmailSendingException
)
from app.schemas.user import UserCreate, UserLogin, EmailVerification, ResendCode
from app.schemas.token import Token


class UserService:
    """Service for user authentication and management."""
    
    def __init__(self, db: Session):
        self.db = db
        self.user_repo = UserRepository(db)

    def authenticate_user(self, email: str, password: str) -> Tuple[Optional[User], bool]:
        """
        Authenticate user with email and password.
        Returns tuple of (user, is_authenticated)
        """
        user = self.user_repo.get_user_by_email(email)
        if not user:
            return None, False
        
        if not verify_password(password, str(user.hashed_password)):
            return None, False
        
        return user, True

    def login_user(self, login_data: UserLogin) -> Token:
        """Login user and return JWT token."""
        user, authenticated = self.authenticate_user(login_data.email, login_data.password)
        
        if not authenticated or not user:
            raise InvalidCredentialsException()
        
        if not user.email_verified_at:
            raise InvalidEmailException()
        
        # Create access token with longer expiration (7 days for better user experience)
        access_token = create_access_token(
            subject=str(user.id),
            expires_delta=timedelta(days=7)
        )
        
        return Token(access_token=access_token)

    async def register_user(self, user_data: UserCreate) -> Tuple[User, str]:
        """Register new user and return user with verification code."""
        try:
            user = self.user_repo.create_user(
                name=user_data.name,
                email=user_data.email,
                password=user_data.password
            )
            
            # Send verification email
            email_sent = await mail_service.send_verification_email(
                email=str(user.email),
                verification_code=str(user.verification_code),
                user_name=str(user.name)
            )
            
            if not email_sent:
                raise EmailSendingException("Failed to send verification email")
                
            return user, str(user.verification_code)
        except UserAlreadyExistsException:
            raise

    def verify_email(self, verification_data: EmailVerification) -> bool:
        """Verify user email with verification code."""
        success = self.user_repo.verify_email(
            email=verification_data.email,
            verification_code=verification_data.verification_code
        )
        
        if not success:
            raise InvalidCredentialsException("Invalid or expired verification code")
        
        return True

    async def resend_verification_code(self, resend_data: ResendCode) -> str:
        """Resend verification code to user email."""
        try:
            user = self.user_repo.get_user_by_email(resend_data.email)
            if not user:
                # Don't reveal if user exists or not for security
                return generate_6_digit_code()  # Return dummy code for security
            
            verification_code = self.user_repo.resend_verification_code(resend_data.email)
            
            # Send verification email
            email_sent = await mail_service.send_verification_email(
                email=str(user.email),
                verification_code=verification_code,
                user_name=str(user.name)
            )
            
            if not email_sent:
                raise EmailSendingException("Failed to send verification email")
                
            return verification_code
        except UserNotFoundException:
            # Don't reveal if user exists or not for security
            return generate_6_digit_code()  # Return dummy code for security

    def get_user_by_id(self, user_id: str) -> Optional[User]:
        """Get user by ID."""
        return self.user_repo.get_user_by_id(user_id)

    def get_user_by_email(self, email: str) -> Optional[User]:
        """Get user by email."""
        return self.user_repo.get_user_by_email(email)

    def update_user_profile(self, user_id: str, name: Optional[str] = None, email: Optional[str] = None) -> User:
        """Update user profile information."""
        update_data: dict[str, Any] = {}
        if name is not None:
            update_data['name'] = name
        if email is not None:
            # Check if email is already taken by another user
            existing_user = self.user_repo.get_user_by_email(email)
            if existing_user and str(existing_user.id) != user_id:
                raise UserAlreadyExistsException(email)
            update_data['email'] = email
            # If email changed, require re-verification
            update_data['email_verified_at'] = None
            update_data['verification_code'] = generate_6_digit_code()
            update_data['verification_code_expires_at'] = datetime.now(timezone.utc) + timedelta(hours=24)

        return self.user_repo.update_user(user_id, **update_data)

    def change_password(self, user_id: str, current_password: str, new_password: str) -> bool:
        """Change user password with current password verification."""
        user = self.user_repo.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)
        
        if not verify_password(current_password, str(user.hashed_password)):
            raise InvalidCredentialsException("Current password is incorrect")
        
        return self.user_repo.change_password(user_id, new_password)

    async def initiate_password_reset(self, email: str) -> str:
        """Initiate password reset process and return reset code."""
        try:
            user = self.user_repo.get_user_by_email(email)
            if not user:
                # Don't reveal if user exists or not for security
                return generate_6_digit_code()  # Return dummy code for security
            
            reset_code = self.user_repo.set_password_reset_code(email)
            
            # Send password reset email
            email_sent = await mail_service.send_password_reset_email(
                email=str(user.email),
                reset_code=reset_code,
                user_name=str(user.name)
            )
            
            if not email_sent:
                raise EmailSendingException("Failed to send password reset email")
                
            return reset_code
        except UserNotFoundException:
            # Don't reveal if user exists or not for security
            return generate_6_digit_code()  # Return dummy code for security

    def complete_password_reset(self, email: str, reset_code: str, new_password: str) -> bool:
        """Complete password reset with reset code."""
        success = self.user_repo.reset_password(email, reset_code, new_password)
        
        if not success:
            raise InvalidCredentialsException("Invalid or expired reset code")
        
        return True

    def is_email_verified(self, user_id: str) -> bool:
        """Check if user's email is verified."""
        user = self.user_repo.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)
        
        return user.email_verified_at is not None

    def get_user_token_data(self, user_id: str) -> dict:
        """Get user data for token payload."""
        user = self.user_repo.get_user_by_id(user_id)
        if not user:
            raise UserNotFoundException(user_id)
        
        return {
            "user_id": user.id,
            "email": user.email,
            "name": user.name,
            "is_admin": user.is_admin,
            "email_verified": user.email_verified_at is not None
        }

    def get_all_users(self, skip: int = 0, limit: int = 100) -> list[User]:
        """Get all users with pagination."""
        return self.user_repo.get_all_users(skip, limit)

    def get_users_count(self) -> int:
        """Get total count of active users."""
        return self.user_repo.get_users_count()

    def delete_user(self, user_id: str) -> bool:
        """Soft delete user account."""
        return self.user_repo.delete_user(user_id)