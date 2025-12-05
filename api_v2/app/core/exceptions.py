from fastapi import HTTPException, status

class UserNotFoundException(HTTPException):
    def __init__(self, user_id: str):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with id {user_id} not found",
        )

class InvalidCredentialsException(HTTPException):
    def __init__(self, detail: str = "Invalid email or password"):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=detail,
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

class EmailSendingException(HTTPException):
    def __init__(self, detail: str = "Failed to send email"):
        super().__init__(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=detail,
        )

class MetricValidationException(HTTPException):
    def __init__(self, detail: str = "Metric validation failed"):
        super().__init__(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=detail,
        )

class MetricRateLimitException(HTTPException):
    def __init__(self, detail: str = "Rate limit exceeded for metric operations"):
        super().__init__(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=detail,
        )

class MetricBatchSizeException(HTTPException):
    def __init__(self, max_size: int = 1000):
        super().__init__(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Batch size exceeds maximum allowed size of {max_size} metrics",
        )

class MetricAccessDeniedException(HTTPException):
    def __init__(self, detail: str = "Access denied to metric"):
        super().__init__(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=detail,
        )
