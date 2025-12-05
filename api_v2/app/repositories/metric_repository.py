from datetime import datetime, timezone, date
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import and_, desc, func
from app.models.metric import Metric
from app.models.user import User
from app.schemas.metric import MetricCreate, MetricUpdate
from app.core.exceptions import MetricNotFoundException, MetricCreationException, UserNotFoundException


class MetricRepository:
    """Repository for metric database operations."""
    
    def __init__(self, db: Session):
        self.db = db

    def _validate_user_exists(self, user_id: str) -> None:
        """Validate that user exists and is active."""
        user = self.db.query(User).filter(
            User.id == user_id,
            User.deleted_at.is_(None)
        ).first()
        if not user:
            raise UserNotFoundException(user_id)

    def create_metric(self, metric_data: MetricCreate) -> Metric:
        """
        Create a single metric for a user.
        
        Args:
            metric_data: Metric creation data
            
        Returns:
            Created metric instance
            
        Raises:
            UserNotFoundException: If user does not exist
            MetricCreationException: If metric creation fails
        """
        try:
            # Validate user exists
            self._validate_user_exists(metric_data.user_id)
            
            # Create metric
            metric = Metric(
                metric_type=metric_data.metric_type,
                value=metric_data.value,
                unit=metric_data.unit,
                sensor_model=metric_data.sensor_model,
                timestamp=metric_data.timestamp or datetime.now(timezone.utc),
                user_id=metric_data.user_id
            )
            
            self.db.add(metric)
            self.db.commit()
            self.db.refresh(metric)
            return metric
            
        except (UserNotFoundException, MetricCreationException):
            # Re-raise specific exceptions to preserve error information
            self.db.rollback()
            raise
        except Exception as e:
            self.db.rollback()
            raise MetricCreationException() from e

    def create_metrics_batch(self, user_id: str, metrics_data: List[MetricCreate]) -> List[Metric]:
        """
        Bulk create multiple metrics for a user.
        
        Args:
            user_id: User ID to associate metrics with
            metrics_data: List of metric creation data
            
        Returns:
            List of created metric instances
            
        Raises:
            UserNotFoundException: If user does not exist
            MetricCreationException: If batch creation fails
        """
        try:
            # Validate user exists
            self._validate_user_exists(user_id)
            
            # Use individual inserts to get proper object persistence
            created_metrics = []
            for metric_data in metrics_data:
                metric = Metric(
                    metric_type=metric_data.metric_type,
                    value=metric_data.value,
                    unit=metric_data.unit,
                    sensor_model=metric_data.sensor_model,
                    timestamp=metric_data.timestamp or datetime.now(timezone.utc),
                    user_id=user_id
                )
                self.db.add(metric)
                created_metrics.append(metric)
            
            self.db.commit()
            
            # Refresh all metrics to get IDs and timestamps
            for metric in created_metrics:
                self.db.refresh(metric)
            
            return created_metrics
            
        except (UserNotFoundException, MetricCreationException):
            # Re-raise specific exceptions to preserve error information
            self.db.rollback()
            raise
        except Exception as e:
            self.db.rollback()
            raise MetricCreationException() from e

    def get_metrics(
        self, 
        metric_type: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None,
        skip: int = 0, 
        limit: int = 100
    ) -> List[Metric]:
        """
        Get metrics for a specific user with optional filtering.
        
        Args:
            user_id: User ID to filter metrics by
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of user metrics
        """
        query = self.db.query(Metric).filter(
            Metric.deleted_at.is_(None)
        )
        
        # Apply filters
        if metric_type:
            query = query.filter(Metric.metric_type == metric_type)
        
        if start_date:
            query = query.filter(Metric.timestamp >= start_date)
        
        if end_date:
            query = query.filter(Metric.timestamp <= end_date)
        
        # Order by timestamp descending (most recent first)
        query = query.order_by(desc(Metric.timestamp))
        
        return query.offset(skip).limit(limit).all()

    def get_user_metrics(
        self, 
        user_id: str, 
        metric_type: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None,
        skip: int = 0, 
        limit: int = 100
    ) -> List[Metric]:
        """
        Get metrics for a specific user with optional filtering.
        
        Args:
            user_id: User ID to filter metrics by
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            skip: Number of records to skip
            limit: Maximum number of records to return
            
        Returns:
            List of user metrics
        """
        query = self.db.query(Metric).filter(
            Metric.user_id == user_id,
            Metric.deleted_at.is_(None)
        )
        
        # Apply filters
        if metric_type:
            query = query.filter(Metric.metric_type == metric_type)
        
        if start_date:
            query = query.filter(Metric.timestamp >= start_date)
        
        if end_date:
            query = query.filter(Metric.timestamp <= end_date)
        
        # Order by timestamp descending (most recent first)
        query = query.order_by(desc(Metric.timestamp))
        
        return query.offset(skip).limit(limit).all()

    def get_metric_by_id(self, metric_id: str, user_id: Optional[str] = None) -> Optional[Metric]:
        """
        Get a specific metric by ID with optional user validation.
        
        Args:
            metric_id: Metric ID to retrieve
            user_id: Optional user ID for validation
            
        Returns:
            Metric instance if found, None otherwise
            
        Raises:
            MetricNotFoundException: If metric not found
        """
        query = self.db.query(Metric).filter(
            Metric.id == metric_id,
            Metric.deleted_at.is_(None)
        )
        
        if user_id:
            query = query.filter(Metric.user_id == user_id)
        
        metric = query.first()
        
        if not metric:
            raise MetricNotFoundException(metric_id)
        
        return metric

    def update_metric(self, metric_id: str, metric_data: MetricUpdate, user_id: Optional[str] = None) -> Metric:
        """
        Update an existing metric.
        
        Args:
            metric_id: Metric ID to update
            metric_data: Metric update data
            user_id: Optional user ID for validation
            
        Returns:
            Updated metric instance
            
        Raises:
            MetricNotFoundException: If metric not found
        """
        metric = self.get_metric_by_id(metric_id, user_id)
        
        # Update fields
        update_fields = metric_data.dict(exclude_unset=True)
        for key, value in update_fields.items():
            if hasattr(metric, key):
                setattr(metric, key, value)
        
        metric.updated_at = datetime.now(timezone.utc)
        self.db.commit()
        self.db.refresh(metric)
        return metric

    def delete_metric(self, metric_id: str, user_id: Optional[str] = None) -> bool:
        """
        Soft delete a metric by setting deleted_at timestamp.
        
        Args:
            metric_id: Metric ID to delete
            user_id: Optional user ID for validation
            
        Returns:
            True if deletion successful
            
        Raises:
            MetricNotFoundException: If metric not found
        """
        metric = self.get_metric_by_id(metric_id, user_id)
        
        metric.deleted_at = datetime.now(timezone.utc)
        self.db.commit()
        return True

    def get_user_metrics_count(
        self, 
        user_id: str, 
        metric_type: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> int:
        """
        Get count of metrics for a specific user with optional filtering.
        
        Args:
            user_id: User ID to filter metrics by
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Count of user metrics
        """
        query = self.db.query(Metric).filter(
            Metric.user_id == user_id,
            Metric.deleted_at.is_(None)
        )
        
        # Apply filters
        if metric_type:
            query = query.filter(Metric.metric_type == metric_type)
        
        if start_date:
            query = query.filter(Metric.timestamp >= start_date)
        
        if end_date:
            query = query.filter(Metric.timestamp <= end_date)
        
        return query.count()

    def get_metrics_summary(
        self,
        user_id: str,
        metric_type: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> dict:
        """
        Get summary statistics for user metrics.
        
        Args:
            user_id: User ID to filter metrics by
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Dictionary with summary statistics
        """
        query = self.db.query(Metric).filter(
            Metric.user_id == user_id,
            Metric.deleted_at.is_(None),
            Metric.value.isnot(None)  # Only include metrics with values
        )
        
        # Apply filters
        if metric_type:
            query = query.filter(Metric.metric_type == metric_type)
        
        if start_date:
            query = query.filter(Metric.timestamp >= start_date)
        
        if end_date:
            query = query.filter(Metric.timestamp <= end_date)
        
        metrics = query.all()
        
        if not metrics:
            return {
                "count": 0,
                "average_value": None,
                "min_value": None,
                "max_value": None,
                "latest_timestamp": None
            }
        
        values = [metric.value for metric in metrics if metric.value is not None]
        timestamps = [metric.timestamp for metric in metrics if metric.timestamp]
        
        return {
            "count": len(metrics),
            "average_value": sum(values) / len(values) if values else None,
            "min_value": min(values) if values else None,
            "max_value": max(values) if values else None,
            "latest_timestamp": max(timestamps) if timestamps else None,
            "unit": metrics[0].unit if metrics else None
        }

    def get_daily_metrics_summary(
        self,
        user_id: str,
        metric_type: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> Dict[str, Any]:
        """
        Get daily summary statistics for user metrics.
        
        Args:
            user_id: User ID to filter metrics by
            metric_type: Optional metric type filter
            start_date: Optional start date filter
            end_date: Optional end date filter
            
        Returns:
            Dictionary with daily summary statistics
        """
        # Base query for metrics with values
        query = self.db.query(
            func.date(Metric.timestamp).label('date'),
            func.avg(Metric.value).label('average_value'),
            func.count(Metric.id).label('count'),
            Metric.metric_type,
            Metric.unit
        ).filter(
            Metric.user_id == user_id,
            Metric.deleted_at.is_(None),
            Metric.value.isnot(None)
        )
        
        # Apply filters
        if metric_type:
            query = query.filter(Metric.metric_type == metric_type)
        
        if start_date:
            query = query.filter(Metric.timestamp >= start_date)
        
        if end_date:
            query = query.filter(Metric.timestamp <= end_date)
        
        # Group by date, metric_type, and unit
        query = query.group_by(
            func.date(Metric.timestamp),
            Metric.metric_type,
            Metric.unit
        ).order_by(func.date(Metric.timestamp))
        
        # Execute query
        daily_summaries = query.all()
        
        # Convert to list of dictionaries
        daily_summaries_list = []
        for summary in daily_summaries:
            daily_summaries_list.append({
                "date": summary.date.isoformat(),
                "average_value": float(summary.average_value) if summary.average_value else None,
                "count": summary.count,
                "metric_type": summary.metric_type,
                "unit": summary.unit
            })
        
        # Calculate total days in range and days with data
        total_days = 0
        days_with_data = len(daily_summaries_list)
        
        if start_date and end_date:
            # Calculate total days in the date range
            start_dt = start_date.date() if isinstance(start_date, datetime) else start_date
            end_dt = end_date.date() if isinstance(end_date, datetime) else end_date
            total_days = (end_dt - start_dt).days + 1
        
        return {
            "daily_summaries": daily_summaries_list,
            "metric_type": metric_type,
            "start_date": start_date,
            "end_date": end_date,
            "total_days": total_days,
            "days_with_data": days_with_data
        }