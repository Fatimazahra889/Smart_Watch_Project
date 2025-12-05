import logging
from typing import Optional
from fastapi_mail import FastMail, MessageSchema, ConnectionConfig
from pydantic import EmailStr
from app.config.settings import settings

# Configure logging
logger = logging.getLogger(__name__)

# Email configuration
email_config = ConnectionConfig(
    MAIL_USERNAME=settings.MAIL_USERNAME,
    MAIL_PASSWORD=settings.MAIL_PASSWORD,
    MAIL_FROM=settings.MAIL_FROM,
    MAIL_PORT=settings.MAIL_PORT,
    MAIL_SERVER=settings.MAIL_SERVER,
    MAIL_FROM_NAME=settings.MAIL_FROM_NAME,
    MAIL_STARTTLS=settings.MAIL_STARTTLS,
    MAIL_SSL_TLS=settings.MAIL_SSL_TLS,
    USE_CREDENTIALS=settings.USE_CREDENTIALS,
    VALIDATE_CERTS=settings.VALIDATE_CERTS,
)

class MailService:
    """Service for sending emails using fastapi-mail."""
    
    def __init__(self):
        self.fast_mail = FastMail(email_config)
    
    async def send_verification_email(self, email: EmailStr, verification_code: str, user_name: str) -> bool:
        """
        Send verification email with 6-digit code.
        
        Args:
            email: Recipient email address
            verification_code: 6-digit verification code
            user_name: User's name for personalization
            
        Returns:
            bool: True if email sent successfully, False otherwise
        """
        try:
            subject = "Verify Your Email - SmartBracelet"
            
            # HTML email template
            html_content = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <style>
                    body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
                    .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
                    .header {{ background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; text-align: center; border-radius: 10px 10px 0 0; }}
                    .content {{ background: #f9f9f9; padding: 30px; border-radius: 0 0 10px 10px; }}
                    .verification-code {{ background: #667eea; color: white; padding: 15px 30px; font-size: 24px; font-weight: bold; text-align: center; border-radius: 5px; margin: 20px 0; letter-spacing: 5px; }}
                    .footer {{ text-align: center; margin-top: 20px; color: #666; font-size: 12px; }}
                </style>
            </head>
            <body>
                <div class="container">
                    <div class="header">
                        <h1>SmartBracelet</h1>
                        <p>Email Verification</p>
                    </div>
                    <div class="content">
                        <h2>Hello {user_name},</h2>
                        <p>Thank you for registering with SmartBracelet! To complete your registration, please use the verification code below:</p>
                        
                        <div class="verification-code">
                            {verification_code}
                        </div>
                        
                        <p>This code will expire in 24 hours.</p>
                        <p>If you didn't create an account with SmartBracelet, please ignore this email.</p>
                        
                        <p>Best regards,<br>The SmartBracelet Team</p>
                    </div>
                    <div class="footer">
                        <p>&copy; 2024 SmartBracelet. All rights reserved.</p>
                    </div>
                </div>
            </body>
            </html>
            """
            
            message = MessageSchema(
                subject=subject,
                recipients=[email],
                body=html_content,
                subtype="html"
            )
            
            await self.fast_mail.send_message(message)
            logger.info(f"Verification email sent successfully to {email}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send verification email to {email}: {str(e)}")
            return False
    
    async def send_password_reset_email(self, email: EmailStr, reset_code: str, user_name: str) -> bool:
        """
        Send password reset email with reset code.
        
        Args:
            email: Recipient email address
            reset_code: 6-digit reset code
            user_name: User's name for personalization
            
        Returns:
            bool: True if email sent successfully, False otherwise
        """
        try:
            subject = "Reset Your Password - SmartBracelet"
            
            # HTML email template
            html_content = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <style>
                    body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
                    .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
                    .header {{ background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; text-align: center; border-radius: 10px 10px 0 0; }}
                    .content {{ background: #f9f9f9; padding: 30px; border-radius: 0 0 10px 10px; }}
                    .reset-code {{ background: #667eea; color: white; padding: 15px 30px; font-size: 24px; font-weight: bold; text-align: center; border-radius: 5px; margin: 20px 0; letter-spacing: 5px; }}
                    .footer {{ text-align: center; margin-top: 20px; color: #666; font-size: 12px; }}
                </style>
            </head>
            <body>
                <div class="container">
                    <div class="header">
                        <h1>SmartBracelet</h1>
                        <p>Password Reset</p>
                    </div>
                    <div class="content">
                        <h2>Hello {user_name},</h2>
                        <p>We received a request to reset your password. Please use the reset code below:</p>
                        
                        <div class="reset-code">
                            {reset_code}
                        </div>
                        
                        <p>This code will expire in 1 hour.</p>
                        <p>If you didn't request a password reset, please ignore this email and your password will remain unchanged.</p>
                        
                        <p>Best regards,<br>The SmartBracelet Team</p>
                    </div>
                    <div class="footer">
                        <p>&copy; 2024 SmartBracelet. All rights reserved.</p>
                    </div>
                </div>
            </body>
            </html>
            """
            
            message = MessageSchema(
                subject=subject,
                recipients=[email],
                body=html_content,
                subtype="html"
            )
            
            await self.fast_mail.send_message(message)
            logger.info(f"Password reset email sent successfully to {email}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send password reset email to {email}: {str(e)}")
            return False

# Create a global instance
mail_service = MailService()