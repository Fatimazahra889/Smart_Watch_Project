from fastapi import HTTPException, status

class UserNotFoundException(HTTPException):
    def __init__(self, user_id: int):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with id {user_id} not found",
        )

class InvalidCredentialsException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
        )

class InvalidEmailException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Email not verified",
        )

class UserAlreadyExistsException(HTTPException):
    def __init__(self, email: str):
        super().__init__(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"User with email {email} already exists",
        )

class DeviceAlreadyExistsException(HTTPException):
    def __init__(self, serial_number: str):
        super().__init__(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Device with serial number {serial_number} already exists",
        )

class MetricCreationException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while creating metrics",
        )

class IssueNotFoundException(HTTPException):
    def __init__(self, issue_id: int):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Issue with id {issue_id} not found",
        )

class DeviceNotFoundException(HTTPException):
    def __init__(self, device_id: int):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Device with id {device_id} not found",
        )

class MetricNotFoundException(HTTPException):
    def __init__(self, metric_id: int):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Metric with id {metric_id} not found",
        )
