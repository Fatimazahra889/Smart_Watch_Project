from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any, Tuple
from sqlalchemy.orm import Session
import logging

from app.models.metric import Metric
from app.models.enums import MetricType
from app.models.user import User
from app.repositories.metric_repository import MetricRepository
from app.core.exceptions import (
    MetricNotFoundException,
    MetricCreationException,
    UserNotFoundException,
    InvalidCredentialsException,
    MetricValidationException,
    MetricRateLimitException,
    MetricBatchSizeException
)
from app.schemas.metric import (
    MetricCreate,
    MetricBatchCreate,
    MetricResponse,
    MetricUpdate,
    MetricSummary,
    DailyMetricsSummaryResponse
)

# Configure logging
logger = logging.getLogger(__name__)


class MetricService:
    """Service for metric business logic and batch operations."""
    
    def __init__(self, db: Session):
        self.db = db
        self.metric_repo = MetricRepository(db)
        
        # Define metric type validation ranges
        self.metric_ranges = {
            MetricType.SPO2: (70.0, 100.0),  # SpO2 percentage
            MetricType.HEART_RATE: (30.0, 220.0),  # Heart rate in BPM
            MetricType.SKIN_TEMPERATURE: (20.0, 45.0),  # Skin temperature in Celsius
            MetricType.AMBIENT_TEMPERATURE: (-10.0, 60.0),  # Ambient temperature in Celsius
            MetricType.STEPS: (0.0, 50000.0),  # Steps count
            MetricType.SLEEP: (0.0, 24.0),  # Sleep duration in hours
        }
        
        # Rate limiting configuration
        self.batch_rate_limit = {
            'max_metrics_per_batch': 1000,
            'max_batches_per_hour': 100,
            'max_metrics_per_hour': 10000
        }

    def _validate_metric_value(self, metric_type: MetricType, value: Optional[float]) -> bool:
        """
        Validate metric value against expected ranges for the metric type.
        
        Args:
            metric_type: Type of metric
            value: Metric value to validate
            
        Returns:
            True if value is valid
            
        Raises:
            ValueError: If value is outside expected range
        """
        if value is None:
            return True  # Some metrics can have null values
            
        if metric_type not in self.metric_ranges:
            logger.warning(f"No validation range defined for metric type: {metric_type}")
            return True
            
        min_val, max_val = self.metric_ranges[metric_type]
        
        if not (min_val <= value <= max_val):
            raise MetricValidationException(
                f"Metric value {value} for type {metric_type} is outside valid range "
                f"[{min_val}, {max_val}]"
            )
            
        return True

    def _validate_timestamp_uniqueness(self, user_id: str, timestamp: datetime, metric_type: MetricType) -> bool:
        """
        Check for duplicate or overlapping timestamps.
        
        Args:
            user_id: User ID
            timestamp: Metric timestamp
            metric_type: Metric type
            
        Returns:
            True if timestamp is unique enough
            
        Note:
            Allows metrics within 1 second to handle rapid measurements
        """
        # Check for existing metrics within 1 second window
        existing_metrics = self.metric_repo.get_user_metrics(
            user_id=user_id,
            metric_type=metric_type,
            start_date=timestamp - timedelta(seconds=1),
            end_date=timestamp + timedelta(seconds=1),
            limit=1
        )
        
        if existing_metrics:
            logger.warning(
                f"Potential duplicate timestamp for user {user_id}, "
                f"metric type {metric_type} at {timestamp}"
            )
            # We allow this but log it for monitoring
            
        return True

    def _pre_validate_batch(self, user_id: str, metrics_data: List[MetricCreate]) -> Tuple[List[MetricCreate], List[Dict[str, Any]]]:
        """
        Pre-validate batch data before database operations.
        
        Args:
            user_id: User ID for the batch
            metrics_data: List of metrics to validate
            
        Returns:
            Tuple of (valid_metrics, validation_errors)
        """
        valid_metrics = []
        validation_errors = []
        
        for i, metric_data in enumerate(metrics_data):
            try:
                # Validate user ID consistency
                if metric_data.user_id != user_id:
                    raise MetricValidationException(f"User ID mismatch in batch at index {i}")
                
                # Validate metric value range
                # self._validate_metric_value(metric_data.metric_type, metric_data.value)
                
                # Validate timestamp uniqueness
                if metric_data.timestamp:
                    self._validate_timestamp_uniqueness(
                        user_id, metric_data.timestamp, metric_data.metric_type
                    )
                
                valid_metrics.append(metric_data)
                
            except Exception as e:
                validation_errors.append({
                    'index': i,
                    'metric_type': metric_data.metric_type.value if metric_data.metric_type else None,
                    'error': str(e),
                    'timestamp': metric_data.timestamp
                })
                logger.warning(f"Batch validation error at index {i}: {e}")
        
        return valid_metrics, validation_errors

    def create_metric(self, metric_data: MetricCreate) -> Metric:
        """
        Create a single metric with business validation.
        
        Args:
            metric_data: Metric creation data
            
        Returns:
            Created metric instance
            
        Raises:
            MetricCreationException: If metric creation fails
            ValueError: If metric validation fails
        """
        try:
            # Business validation
            #self._validate_metric_value(metric_data.metric_type, metric_data.value)
            
            # if metric_data.timestamp:
            #     self._validate_timestamp_uniqueness(
            #         metric_data.user_id, metric_data.timestamp, metric_data.metric_type
            #     )
            
            # Create metric via repository
            metric = self.metric_repo.create_metric(metric_data)
            logger.info(f"Created metric {metric.id} for user {metric_data.user_id}")
            
            return metric
            
        except Exception as e:
            logger.error(f"Failed to create metric for user {metric_data.user_id}: {e}")
            if isinstance(e, (MetricCreationException, UserNotFoundException, MetricValidationException)):
                raise
            raise MetricCreationException() from e

    def create_metrics_batch(self, user_id: str, batch_data: MetricBatchCreate) -> Dict[str, Any]:
        """
        Process batch metric creation with validation and error handling.
        
        Args:
            user_id: User ID for the batch
            batch_data: Batch metric creation data
            
        Returns:
            Dictionary with batch operation results
            
        Raises:
            MetricCreationException: If batch creation fails
            ValueError: If batch validation fails
        """
        logger.info(f"Starting batch creation for user {user_id} with {len(batch_data.metrics)} metrics")
        
        try:
            # Pre-validate all metrics in the batch
            valid_metrics, validation_errors = self._pre_validate_batch(
                user_id, batch_data.metrics
            )
            
            if not valid_metrics:
                logger.error(f"No valid metrics in batch for user {user_id}")
                raise MetricCreationException()
            
            # Create valid metrics via repository
            created_metrics = self.metric_repo.create_metrics_batch(user_id, valid_metrics)
            
            # Prepare response
            result = {
                'total_processed': len(batch_data.metrics),
                'successful': len(created_metrics),
                'failed': len(validation_errors),
                'validation_errors': validation_errors,
                'created_metrics': [
                    {
                        'id': metric.id,
                        'metric_type': metric.metric_type.value,
                        'timestamp': metric.timestamp.isoformat() if metric.timestamp else None
                    }
                    for metric in created_metrics
                ]
            }
            
            logger.info(
                f"Batch creation completed for user {user_id}: "
                f"{result['successful']} successful, {result['failed']} failed"
            )
            
            return result
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            logger.error(f"Batch creation failed for user {user_id}: {str(e)}")
            logger.error(f"Traceback: {error_details}")
            if isinstance(e, (MetricCreationException, UserNotFoundException, MetricValidationException)):
                raise
            raise e

    def get_user_metrics(
        self, 
        user: User, 
        metric_type: Optional[MetricType] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None,
        skip: int = 0, 
        limit: int = 100
    ) -> List[Metric]:
        """
        Get metrics for a user with business logic.
        
        Args:
            user_id: User ID to get metrics for
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of user metrics
            
        Raises:
            UserNotFoundException: If user does not exist
        """
        try:
            if user.is_admin:
                # return all metrics
                metrics = self.metric_repo.get_metrics(
                    metric_type=metric_type,
                    start_date=start_date,
                    end_date=end_date,
                    skip=skip,
                    limit=limit
                )

                logger.debug(f"Retrieved {len(metrics)} metrics for user {user.id}")
                return metrics

            metrics = self.metric_repo.get_user_metrics(
                user_id=str(user.id),
                metric_type=metric_type,
                start_date=start_date,
                end_date=end_date,
                skip=skip,
                limit=limit
            )
            
            logger.debug(f"Retrieved {len(metrics)} metrics for user {user.id}")
            return metrics
            
        except UserNotFoundException:
            raise
        except Exception as e:
            logger.error(f"Failed to get metrics for user {user.id}: {e}")
            raise

    def get_metric(self, metric_id: str, user_id: Optional[str] = None) -> Metric:
        """
        Get a specific metric with user validation.
        
        Args:
            metric_id: Metric ID to retrieve
            user_id: Optional user ID for validation
            
        Returns:
            Metric instance
            
        Raises:
            MetricNotFoundException: If metric not found
            InvalidCredentialsException: If user validation fails
        """
        try:
            metric = self.metric_repo.get_metric_by_id(metric_id, user_id)
            
            if user_id and metric and metric.user_id != user_id:
                raise InvalidCredentialsException("User does not have access to this metric")
            
            return metric
            
        except MetricNotFoundException:
            raise
        except Exception as e:
            logger.error(f"Failed to get metric {metric_id}: {e}")
            raise

    def update_metric(self, metric_id: str, metric_data: MetricUpdate, user_id: Optional[str] = None) -> Metric:
        """
        Update a metric with business validation.
        
        Args:
            metric_id: Metric ID to update
            metric_data: Metric update data
            user_id: Optional user ID for validation
            
        Returns:
            Updated metric instance
            
        Raises:
            MetricNotFoundException: If metric not found
            ValueError: If metric validation fails
        """
        try:
            # Get existing metric for validation
            existing_metric = self.get_metric(metric_id, user_id)
            
            # Business validation for updated fields
            update_fields = metric_data.dict(exclude_unset=True)
            
            if 'value' in update_fields and 'metric_type' in update_fields:
                self._validate_metric_value(
                    update_fields['metric_type'], update_fields['value']
                )
            elif 'value' in update_fields:
                self._validate_metric_value(
                    MetricType(existing_metric.metric_type), update_fields['value']
                )
            elif 'metric_type' in update_fields:
                self._validate_metric_value(
                    update_fields['metric_type'], existing_metric.value
                )
            
            if 'timestamp' in update_fields and update_fields['timestamp']:
                self._validate_timestamp_uniqueness(
                    str(existing_metric.user_id), update_fields['timestamp'],
                    update_fields.get('metric_type', existing_metric.metric_type)
                )
            
            # Update metric via repository
            updated_metric = self.metric_repo.update_metric(metric_id, metric_data, user_id)
            logger.info(f"Updated metric {metric_id}")
            
            return updated_metric
            
        except (MetricNotFoundException, InvalidCredentialsException, MetricValidationException):
            raise
        except Exception as e:
            logger.error(f"Failed to update metric {metric_id}: {e}")
            raise

    def delete_metric(self, metric_id: str, user_id: Optional[str] = None) -> bool:
        """
        Delete a metric with proper cleanup logic.
        
        Args:
            metric_id: Metric ID to delete
            user_id: Optional user ID for validation
            
        Returns:
            True if deletion successful
            
        Raises:
            MetricNotFoundException: If metric not found
        """
        try:
            # Validate metric exists and user has access
            metric = self.get_metric(metric_id, user_id)
            
            # Perform deletion via repository
            success = self.metric_repo.delete_metric(metric_id, user_id)
            
            if success:
                logger.info(f"Deleted metric {metric_id} for user {metric.user_id}")
                # Here you could add additional cleanup logic if needed
                # e.g., update analytics, notify systems, etc.
            
            return success
            
        except (MetricNotFoundException, InvalidCredentialsException):
            raise
        except Exception as e:
            logger.error(f"Failed to delete metric {metric_id}: {e}")
            raise

    def get_metrics_summary(
        self,
        user_id: str,
        metric_type: Optional[MetricType] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> Dict[str, Any]:
        """
        Get summary statistics for user metrics with business logic.
        
        Args:
            user_id: User ID to get summary for
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Dictionary with summary statistics
        """
        try:
            summary = self.metric_repo.get_metrics_summary(
                user_id=user_id,
                metric_type=metric_type,
                start_date=start_date,
                end_date=end_date
            )
            
            logger.debug(f"Generated summary for user {user_id}, metric type {metric_type}")
            return summary
            
        except Exception as e:
            logger.error(f"Failed to get metrics summary for user {user_id}: {e}")
            raise

    def get_daily_metrics_summary(
        self,
        user_id: str,
        metric_type: Optional[MetricType] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> Dict[str, Any]:
        """
        Get daily summary statistics for user metrics with business logic.
        
        Args:
            user_id: User ID to get summary for
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Dictionary with daily summary statistics
        """
        try:
            daily_summary = self.metric_repo.get_daily_metrics_summary(
                user_id=user_id,
                metric_type=metric_type,
                start_date=start_date,
                end_date=end_date
            )
            
            logger.debug(f"Generated daily summary for user {user_id}, metric type {metric_type}")
            return daily_summary
            
        except Exception as e:
            logger.error(f"Failed to get daily metrics summary for user {user_id}: {e}")
            raise

    def get_user_metrics_count(
        self, 
        user_id: str, 
        metric_type: Optional[MetricType] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> int:
        """
        Get count of metrics for a user with business logic.
        
        Args:
            user_id: User ID to count metrics for
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Count of user metrics
        """
        try:
            count = self.metric_repo.get_user_metrics_count(
                user_id=user_id,
                metric_type=metric_type,
                start_date=start_date,
                end_date=end_date
            )
            
            logger.debug(f"Counted {count} metrics for user {user_id}")
            return count
            
        except Exception as e:
            logger.error(f"Failed to count metrics for user {user_id}: {e}")
            raise