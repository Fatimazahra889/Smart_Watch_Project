from app.services.blacklist_service import blacklist_service
from app.api.deps import security_scheme
from fastapi.security.http import HTTPAuthorizationCredentials
from app.api.deps import get_current_active_user
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any

from app.api.deps import get_db
from app.services.user_service import UserService
from app.schemas.user import (
    UserCreate, 
    UserLogin, 
    EmailVerification, 
    ResendCode,
    UserResponse
)
from app.schemas.token import Token
from app.core.exceptions import (
    UserNotFoundException,
    InvalidCredentialsException,
    InvalidEmailException,
    UserAlreadyExistsException
)

router = APIRouter()


@router.post("/login", response_model=Token, status_code=status.HTTP_200_OK)
async def login(
    login_data: UserLogin,
    db: Session = Depends(get_db)
) -> Token:
    """
    Authenticate user and return JWT token.
    
    Args:
        login_data: User login credentials (email and password)
        db: Database session dependency
        
    Returns:
        Token: JWT access token with bearer type
        
    Raises:
        HTTPException: 401 if credentials are invalid
        HTTPException: 403 if email is not verified
    """
    try:
        user_service = UserService(db)
        token = user_service.login_user(login_data)
        return token
    except InvalidCredentialsException as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )
    except InvalidEmailException as e:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Email not verified. Please verify your email before logging in."
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during login"
        )

@router.get("/me", response_model=UserResponse, status_code=status.HTTP_200_OK)
async def read_current_user(
    current_user = Depends(get_current_active_user)
) -> UserResponse:
    """
    Get current authenticated user information.
    
    Args:
        current_user: Current authenticated user dependency
        
    Returns:
        UserResponse: Current user information
    """
    return UserResponse.model_validate(current_user)


@router.post("/register", response_model=Dict[str, Any], status_code=status.HTTP_201_CREATED)
async def register(
    user_data: UserCreate,
    db: Session = Depends(get_db)
) -> Dict[str, Any]:
    """
    Register a new user and send verification code.
    
    Args:
        user_data: User registration data (name, email, password)
        db: Database session dependency
        
    Returns:
        Dict: User information and verification status
        
    Raises:
        HTTPException: 409 if user already exists
        HTTPException: 500 for database errors
    """
    try:
        user_service = UserService(db)
        user, verification_code = await user_service.register_user(user_data)
        
        # In a real application, you would send the verification code via email
        # For now, we'll return it in the response (in production, remove this)
        
        return {
            "message": "User registered successfully. Please check your email for verification code.",
            "user": UserResponse.from_orm(user),
            # "verification_code": verification_code,  # Remove in production
            "email_sent": True
        }
    except UserAlreadyExistsException as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"User with email {user_data.email} already exists"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during registration: " + str(e)
        )


@router.post("/verify-email", response_model=Dict[str, str], status_code=status.HTTP_200_OK)
async def verify_email(
    verification_data: EmailVerification,
    db: Session = Depends(get_db)
) -> Dict[str, str]:
    """
    Verify user email with verification code.
    
    Args:
        verification_data: Email and verification code
        db: Database session dependency
        
    Returns:
        Dict: Success message
        
    Raises:
        HTTPException: 400 if verification code is invalid or expired
        HTTPException: 404 if user not found
        HTTPException: 500 for database errors
    """
    try:
        user_service = UserService(db)
        success = user_service.verify_email(verification_data)
        
        if success:
            return {
                "message": "Email verified successfully. You can now log in."
            }
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid or expired verification code"
            )
            
    except InvalidCredentialsException as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired verification code"
        )
    except UserNotFoundException as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during email verification"
        )


@router.post("/resend-code", response_model=Dict[str, str], status_code=status.HTTP_200_OK)
async def resend_verification_code(
    resend_data: ResendCode,
    db: Session = Depends(get_db)
) -> Dict[str, str]:
    """
    Resend verification code to user email.
    
    Args:
        resend_data: Email to resend verification code to
        db: Database session dependency
        
    Returns:
        Dict: Success message (always returns success for security)
        
    Note:
        For security reasons, this endpoint always returns success
        even if the email doesn't exist to prevent email enumeration.
    """
    try:
        user_service = UserService(db)
        verification_code = await user_service.resend_verification_code(resend_data)
        
        # In a real application, you would send the verification code via email
        # For now, we'll return it in the response (in production, remove this)
        
        return {
            "message": "Verification code sent successfully. Please check your email.",
            "verification_code": verification_code  # Remove in production
        }
    except Exception as e:
        # Always return success for security reasons
        return {
            "message": "Verification code sent successfully. Please check your email."
        }


@router.post("/logout", response_model=Dict[str, str], status_code=status.HTTP_200_OK)
async def logout(
    *,
    current_user = Depends(get_current_active_user),
    credentials: HTTPAuthorizationCredentials = Depends(security_scheme)
) -> Any:
    """
    Logout user (token invalidation).
    
    Note:
        For stateless JWT, logout is typically handled client-side by
        removing the token from storage. This endpoint can be used for
        server-side token blacklisting if needed in the future.
        
    Returns:
        Dict: Success message
    """

    try:
        token = credentials.credentials
        
        # Extract JTI from token
        jti = blacklist_service.get_token_jti(token)
        if not jti:
            raise HTTPException(status_code=400, detail="Invalid token format")
        
        # Get token expiration time
        expires_at = blacklist_service.get_token_expiration(token)
        if not expires_at:
            raise HTTPException(status_code=400, detail="Invalid token format")
        
        # Add token to blacklist
        success = blacklist_service.add_to_blacklist(jti, expires_at)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to logout")
        
        return {"message": "Successfully logged out"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail="Internal server error during logout")