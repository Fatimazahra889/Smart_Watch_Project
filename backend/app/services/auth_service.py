from sqlalchemy.orm import Session
from app.models.user import User
from app.services.user_service import user_service
from app.core.security import verify_password
from app.core.exceptions import InvalidCredentialsException ,InvalidEmailException

class AuthService:
    def authenticate_user(
        self, db: Session, *, email: str, password: str
    ) -> User | None:
        user = user_service.get_by_email(db, email=email)
        if not user or not verify_password(password, user.hashed_password):
            raise InvalidCredentialsException()
        self.verify_email(user)
        return user

    def verify_email(self, user: User) -> None:
        if user.email_verified_at is None:
            raise InvalidEmailException()

auth_service = AuthService()