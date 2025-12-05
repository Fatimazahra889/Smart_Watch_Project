from sqlalchemy.orm import Session
from app.models.user import User
from app.schemas.user import UserCreate, UserUpdate
from app.repositories.user_repository import user_repository
from app.repositories.metric_repository import metric_repository
from app.core.security import get_password_hash, generate_6_digit_code
from datetime import datetime, timedelta
from app.core.exceptions import UserNotFoundException, UserAlreadyExistsException, InvalidCredentialsException
import smtplib, ssl
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from app.config.settings import settings
from typing import List, Dict, Any
from uuid import UUID
from app.models.enums import AggregationPeriod, MetricType
from app.models.metric import Metric

class UserService:
    def __init__(self, user_repo):
        self.user_repo = user_repo

    def get(self, db: Session, id: str) -> User | None:
        user = self.user_repo.get(db, id=id)
        if not user:
            raise UserNotFoundException(user_id=id)
        return user

    def get_multi(
        self, db: Session, *, skip: int = 0, limit: int = 100
    ) -> List[User]:
        return self.user_repo.get_multi(db, skip=skip, limit=limit)

    def create_user(self, db: Session, *, user_in: UserCreate) -> User:
        existing_user = self.get_by_email(db, email=user_in.email)
        if existing_user:
            raise UserAlreadyExistsException(email=user_in.email)
        
        hashed_password = get_password_hash(user_in.password)
        verification_code = generate_6_digit_code()
        
        user_data = user_in.model_dump()
        user_data["hashed_password"] = hashed_password
        user_data["verification_code"] = verification_code
        user_data["verification_code_expires_at"] = datetime.utcnow() + timedelta(hours=1)
        del user_data["password"]

        user = self.user_repo.create(db, obj_in=user_data)
        self.send_verification_email(user, verification_code)
        return user

    def update_user(
        self, db: Session, *, db_obj: User, obj_in: UserUpdate | Dict[str, Any]
    ) -> User:
        if isinstance(obj_in, dict):
            update_data = obj_in
        else:
            update_data = obj_in.model_dump(exclude_unset=True)

        if "password" in update_data and update_data["password"]:
            hashed_password = get_password_hash(update_data["password"])
            update_data["hashed_password"] = hashed_password
            del update_data["password"]
        
        return self.user_repo.update(db, db_obj=db_obj, obj_in=update_data)

    def delete_user(self, db: Session, *, id: str) -> User | None:
        user = self.user_repo.remove(db, id=id)
        if not user:
            raise UserNotFoundException(user_id=id)
        return user

    def get_by_email(self, db: Session, *, email: str) -> User | None:
        return self.user_repo.get_by_email(db, email=email)

    def send_verification_email(self, user: User, verification_code: str):
        message = MIMEMultipart("alternative")
        message["Subject"] = "Verify your email address"
        message["From"] = settings.SMTP_USER
        message["To"] = user.email

        text = f"""\
        Hi {user.name},
        Thanks for signing up to our service.
        Your verification code is: {verification_code}
        """
        html = f"""\
        <html>
          <body>
            <p>Hi {user.name},<br>
               Thanks for signing up to our service.<br>
               Your verification code is: <b>{verification_code}</b>
            </p>
          </body>
        </html>
        """

        part1 = MIMEText(text, "plain")
        part2 = MIMEText(html, "html")

        message.attach(part1)
        message.attach(part2)

        context = ssl.create_default_context()
        with smtplib.SMTP_SSL(settings.SMTP_SERVER, settings.SMTP_PORT, context=context) as server:
            server.login(settings.SMTP_USER, settings.SMTP_PASSWORD)
            server.sendmail(
                settings.SMTP_USER, user.email, message.as_string()
            )

    def verify_email_with_code(self, db: Session, *, email: str, code: str) -> User | None:
        user = self.get_by_email(db, email=email)
        if (
            not user
            or not user.verification_code
            or user.verification_code != code
            or user.verification_code_expires_at < datetime.utcnow()
        ):
            raise InvalidCredentialsException(detail="Invalid or expired verification code.")

        user.email_verified_at = datetime.utcnow()
        user.verification_code = None
        user.verification_code_expires_at = None
        db.add(user)
        db.commit()
        db.refresh(user)
        return user

    def initiate_password_reset(self, db: Session, *, email: str):
        user = self.get_by_email(db, email=email)
        if not user:
            raise UserNotFoundException()

        password_reset_code = generate_6_digit_code()
        user.password_reset_code = password_reset_code
        user.password_reset_code_expires_at = datetime.utcnow() + timedelta(minutes=15)
        db.add(user)
        db.commit()
        self.send_password_reset_email(user, password_reset_code)

    def send_password_reset_email(self, user: User, password_reset_code: str):
        message = MIMEMultipart("alternative")
        message["Subject"] = "Reset your password"
        message["From"] = settings.SMTP_USER
        message["To"] = user.email

        text = f"""\
        Hi {user.name},
        Your password reset code is: {password_reset_code}
        """
        html = f"""\
        <html>
          <body>
            <p>Hi {user.name},<br>
               Your password reset code is: <b>{password_reset_code}</b>
            </p>
          </body>
        </html>
        """

        part1 = MIMEText(text, "plain")
        part2 = MIMEText(html, "html")

        message.attach(part1)
        message.attach(part2)

        context = ssl.create_default_context()
        with smtplib.SMTP_SSL(settings.SMTP_SERVER, settings.SMTP_PORT, context=context) as server:
            server.login(settings.SMTP_USER, settings.SMTP_PASSWORD)
            server.sendmail(
                settings.SMTP_USER, user.email, message.as_string()
            )

    def reset_password_with_code(self, db: Session, *, email: str, code: str, new_password: str) -> User:
        user = self.get_by_email(db, email=email)
        if (
            not user
            or not user.password_reset_code
            or user.password_reset_code != code
            or user.password_reset_code_expires_at < datetime.utcnow()
        ):
            raise InvalidCredentialsException(detail="Invalid or expired password reset code.")

        user.hashed_password = get_password_hash(new_password)
        user.password_reset_code = None
        user.password_reset_code_expires_at = None
        db.add(user)
        db.commit()
        db.refresh(user)
        return user

    def get_metrics_summary(
        self, db: Session, *, user_id: UUID, period: AggregationPeriod, metric_type: MetricType
    ) -> List[Dict[str, Any]]:
        return metric_repository.get_summary(
            db,
            user_id=user_id,
            period=period.value,
            metric_type=metric_type,
        )

    def get_metrics_by_type(
        self, db: Session, *, user_id: UUID, metric_type: MetricType
    ) -> List[Metric]:
        return metric_repository.get_by_user_and_type(
            db,
            user_id=user_id,
            metric_type=metric_type,
        )

user_service = UserService(user_repository)