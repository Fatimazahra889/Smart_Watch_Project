from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.api import deps
from app.models.enums import AggregationPeriod, MetricType
from app.schemas.summary import MetricsSummaryResponse
from app.services.user_service import user_service
from uuid import UUID
from app.models.user import User
from app.schemas.metric import Metric
from typing import List

router = APIRouter()

@router.get("/users/{user_id}/metrics/summary", response_model=MetricsSummaryResponse)
def get_metrics_summary(
    user_id: UUID,
    metric_type: MetricType,
    period: AggregationPeriod,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Get aggregated metrics for a user.
    """
    if current_user.id != user_id and not current_user.is_admin:
        raise HTTPException(status_code=403, detail="Not authorized to access this resource")

    summary = user_service.get_metrics_summary(
        db,
        user_id=user_id,
        metric_type=metric_type,
        period=period,
    )
    if not summary:
        raise HTTPException(status_code=404, detail="No metrics found for this user.")
    return {"metrics": summary}

@router.get("/users/{user_id}/metrics/data", response_model=List[Metric])
def get_metrics_data(
    user_id: UUID,
    metric_type: MetricType,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Get raw metric data for a user, filtered by metric_type.
    """
    if current_user.id != user_id and not current_user.is_admin:
        raise HTTPException(status_code=403, detail="Not authorized to access this resource")

    metrics = user_service.get_metrics_by_type(
        db,
        user_id=user_id,
        metric_type=metric_type,
    )
    if not metrics:
        raise HTTPException(status_code=404, detail="No metrics found for this user.")
    return metrics
