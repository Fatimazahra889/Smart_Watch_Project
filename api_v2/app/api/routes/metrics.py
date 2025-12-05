from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any, List, Optional
import logging

from app.api.deps import get_current_user, get_db, get_current_admin_user
from app.services.metric_service import MetricService
from app.services.health_prediction_service import HealthPredictionService
from app.schemas.metric import MetricBatchCreate, MetricResponse, MetricSummary, DailyMetricsSummaryResponse
from app.schemas.health_prediction import HealthPredictionResponse, HealthPredictionRequest
from app.core.exceptions import (
    MetricCreationException,
    UserNotFoundException,
    InvalidCredentialsException,
    MetricNotFoundException,
    MetricValidationException,
    MetricRateLimitException,
    MetricBatchSizeException,
    MetricAccessDeniedException
)
from app.models.user import User
from app.models.enums import MetricType

# Configure logging
logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/batch", response_model=Dict[str, Any], status_code=status.HTTP_201_CREATED)
async def create_metrics_batch(
    batch_data: MetricBatchCreate,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> Dict[str, Any]:
    """
    Store batch metrics for authenticated user.
    
    This endpoint allows authenticated users to store multiple metrics in a single request.
    The batch can contain up to 1000 metrics and includes comprehensive validation
    for each metric in the batch.
    
    Args:
        batch_data: Batch metric creation data containing list of metrics
        current_user: Authenticated user (from JWT token)
        db: Database session dependency
        
    Returns:
        Dict: Batch operation results including:
            - total_processed: Total number of metrics in the batch
            - successful: Number of successfully created metrics
            - failed: Number of metrics that failed validation
            - validation_errors: Detailed error information for failed metrics
            - created_metrics: List of successfully created metrics with IDs
            
    Raises:
        HTTPException: 400 if batch validation fails
        HTTPException: 401 if authentication fails
        HTTPException: 500 if batch creation fails
    """
    try:
        metric_service = MetricService(db)
        
        # Set user_id for all metrics in the batch from authenticated user
        for metric in batch_data.metrics:
            metric.user_id = str(current_user.id)
        
        # Process batch creation with user ID from authenticated user
        result = metric_service.create_metrics_batch(
            user_id=str(current_user.id),
            batch_data=batch_data
        )
        
        return {
            "message": "Batch metrics processed successfully",
            "data": result
        }
        
    except MetricCreationException as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e) or "Failed to create metrics batch" + str(e)
        )
    except UserNotFoundException as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    except InvalidCredentialsException as e:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied to requested resources"
        )
    except MetricValidationException as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=str(e) or "Metric validation failed"
        )
    except MetricRateLimitException as e:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=str(e) or "Rate limit exceeded"
        )
    except MetricBatchSizeException as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e) or "Batch size too large"
        )
    except Exception as e:
        import traceback
        error_details = traceback.format_exc()
        logger.error(f"Batch metrics creation failed: {str(e)}")
        logger.error(f"Traceback: {error_details}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An error occurred while processing batch metrics: {str(e)}"
        )


@router.get("/", response_model=List[MetricResponse])
async def get_metrics(
    skip: int = 0,
    limit: int = 100,
    metric_type: Optional[MetricType] = None,
    start_date: Optional[str] = None,
    end_date: Optional[str] = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> List[MetricResponse]:
    """
    Get all metrics (admin only).
    
    This endpoint allows admin users to retrieve all metrics in the system
    with optional filtering by metric type and date range.
    
    Args:
        skip: Number of records to skip
        limit: Maximum number of records to return
        metric_type: Optional metric type filter
        start_date: Optional start date filter (ISO format)
        end_date: Optional end date filter (ISO format)
        current_user: Authenticated admin user
        db: Database session dependency
        
    Returns:
        List of metric responses
        
    Raises:
        HTTPException: 401 if authentication fails
        HTTPException: 403 if user is not admin
        HTTPException: 500 if retrieval fails
    """
    try:
        metric_service = MetricService(db)
        
        # Parse date strings to datetime objects
        from datetime import datetime
        start_dt = datetime.fromisoformat(start_date) if start_date else None
        end_dt = datetime.fromisoformat(end_date) if end_date else None
        
        metrics = metric_service.get_user_metrics(
            user=current_user,  # Admin can see all metrics
            metric_type=metric_type,
            start_date=start_dt,
            end_date=end_dt,
            skip=skip,
            limit=limit
        )
        
        return [MetricResponse.from_orm(metric) for metric in metrics]
        
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving metrics"
        )

@router.get("/health-prediction", response_model=HealthPredictionResponse)
async def get_health_prediction(
    include_metrics: bool = False,
    prediction_horizon_hours: Optional[int] = 24,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> HealthPredictionResponse:
    """
    Get health prediction based on last 24 hours of metrics.
    
    This endpoint provides a comprehensive health assessment based on the user's
    metrics data from the last 24 hours, including:
    - Overall health score (0-1)
    - Health risk level (LOW, MEDIUM, HIGH)
    - Confidence score
    - Individual metric analysis
    - Risk factors and recommendations
    
    Args:
        include_metrics: Include raw metrics summary in response
        prediction_horizon_hours: Prediction horizon in hours (1-168, default: 24)
        current_user: Authenticated user
        db: Database session dependency
        
    Returns:
        Health prediction response with comprehensive analysis
        
    Raises:
        HTTPException: 401 if authentication fails
        HTTPException: 500 if prediction generation fails
    """
    try:
        health_prediction_service = HealthPredictionService(db)
        
        # Create prediction request
        request = HealthPredictionRequest(
            user_id=str(current_user.id),
            include_metrics=include_metrics,
            prediction_horizon_hours=prediction_horizon_hours
        )
        
        # Get health prediction
        prediction = health_prediction_service.predict_health_status(request)
        
        return prediction
        
    except UserNotFoundException as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while generating health prediction"
        )


@router.get("/summary", response_model=Dict[str, Any])
async def get_metrics_summary(
    metric_type: Optional[MetricType] = None,
    start_date: Optional[str] = None,
    end_date: Optional[str] = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> Dict[str, Any]:
    """
    Get aggregated metrics summary with statistics.
    
    This endpoint provides aggregated summary statistics (count, average, min, max)
    for the authenticated user's metrics with optional filtering by metric type
    and date range.
    
    Args:
        metric_type: Optional metric type filter (spo2, heart_rate, skin_temperature, steps, sleep)
        start_date: Optional start date filter (ISO format)
        end_date: Optional end date filter (ISO format)
        current_user: Authenticated user
        db: Database session dependency
        
    Returns:
        Dictionary with aggregated summary statistics for the specified metric type
        
    Raises:
        HTTPException: 401 if authentication fails
        HTTPException: 500 if summary generation fails
    """
    try:
        metric_service = MetricService(db)
        
        # Parse date strings to datetime objects
        from datetime import datetime
        start_dt = datetime.fromisoformat(start_date) if start_date else None
        end_dt = datetime.fromisoformat(end_date) if end_date else None
        
        summary = metric_service.get_metrics_summary(
            user_id=str(current_user.id),
            metric_type=metric_type,
            start_date=start_dt,
            end_date=end_dt
        )
        
        # Format the response to match frontend expectations
        if metric_type:
            # Single metric type requested
            return {
                metric_type.value: {
                    "count": summary.get("count", 0),
                    "average": summary.get("average_value"),
                    "min": summary.get("min_value"),
                    "max": summary.get("max_value"),
                    "std_dev": None  # Standard deviation not currently calculated
                }
            }
        else:
            # No specific metric type - return empty summary
            return {}
        
    except Exception as e:
        logger.error(f"Error generating metrics summary: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while generating metrics summary"
        )


@router.get("/summary/daily", response_model=DailyMetricsSummaryResponse)
async def get_user_metrics_summary(
    metric_type: Optional[MetricType] = None,
    start_date: Optional[str] = None,
    end_date: Optional[str] = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> DailyMetricsSummaryResponse:
    """
    Get daily summary statistics for user metrics.
    
    This endpoint provides daily summary statistics (average value, count)
    for the authenticated user's metrics with optional filtering.
    Returns daily averages for each day within the specified date range.
    
    Args:
        metric_type: Optional metric type filter
        start_date: Optional start date filter (ISO format)
        end_date: Optional end date filter (ISO format)
        current_user: Authenticated user
        db: Database session dependency
        
    Returns:
        Daily metrics summary response containing daily averages
        
    Raises:
        HTTPException: 401 if authentication fails
        HTTPException: 500 if summary generation fails
    """
    try:
        metric_service = MetricService(db)
        
        # Parse date strings to datetime objects
        from datetime import datetime
        start_dt = datetime.fromisoformat(start_date) if start_date else None
        end_dt = datetime.fromisoformat(end_date) if end_date else None
        
        daily_summary = metric_service.get_daily_metrics_summary(
            user_id=str(current_user.id),
            metric_type=metric_type,
            start_date=start_dt,
            end_date=end_dt
        )
        
        return DailyMetricsSummaryResponse(**daily_summary)
        
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while generating metrics summary"
        )


@router.get("/{metric_id}", response_model=MetricResponse)
async def get_metric(
    metric_id: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
) -> MetricResponse:
    """
    Get a specific metric by ID.
    
    This endpoint allows authenticated users to retrieve a specific metric
    by its UUID. Users can only access their own metrics.
    
    Args:
        metric_id: Metric UUID to retrieve
        current_user: Authenticated user
        db: Database session dependency
        
    Returns:
        Metric response
        
    Raises:
        HTTPException: 404 if metric not found
        HTTPException: 403 if user doesn't have access to the metric
        HTTPException: 500 if retrieval fails
    """
    try:
        metric_service = MetricService(db)
        metric = metric_service.get_metric(metric_id, str(current_user.id))
        return MetricResponse.from_orm(metric)
        
    except MetricNotFoundException as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e) or "Metric not found"
        )
    except MetricAccessDeniedException as e:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=str(e) or "Access denied to metric"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving the metric"
        )


@router.delete("/{metric_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_metric(
    metric_id: str,
    current_user: User = Depends(get_current_admin_user),
    db: Session = Depends(get_db)
) -> None:
    """
    Delete a metric (admin only).
    
    This endpoint allows admin users to delete a specific metric by its UUID.
    
    Args:
        metric_id: Metric UUID to delete
        current_user: Authenticated admin user
        db: Database session dependency
        
    Raises:
        HTTPException: 404 if metric not found
        HTTPException: 403 if user is not admin
        HTTPException: 500 if deletion fails
    """
    try:
        metric_service = MetricService(db)
        metric_service.delete_metric(metric_id)
        
    except MetricNotFoundException as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e) or "Metric not found"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while deleting the metric"
        )