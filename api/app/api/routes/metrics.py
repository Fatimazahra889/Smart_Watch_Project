from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.api import deps
from app.models.device import Device
from app.models.user import User
from app.schemas.metric import Metric, MetricBatch
from app.services.metric_service import metric_service
from typing import List
from uuid import UUID

router = APIRouter()

@router.post("/metrics/batch/", status_code=201)
def create_metrics(
    *,
    db: Session = Depends(deps.get_db),
    current_device: Device = Depends(deps.get_current_device),
    metrics_in: MetricBatch,
) -> dict:
    """
    Create new metrics for the current device.
    """
    metric_service.create_metrics(db, device=current_device, metrics_in=metrics_in)
    return {"message": "Metrics received"}

@router.get("/metrics/", response_model=List[Metric])
def get_metrics(
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
    skip: int = 0,
    limit: int = 100,
):
    """
    Retrieve all metrics (admin only).
    """
    if not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this resource."
        )
    metrics = metric_service.get_metrics(db, skip=skip, limit=limit)
    return metrics

@router.get("/metrics/{metric_id}", response_model=Metric)
def get_metric(
    metric_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Get a specific metric by ID.
    """
    metric = metric_service.get_metric(db, metric_id=metric_id)
    if not current_user.is_admin and metric.user_id != current_user.id:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this metric."
        )
    return metric

@router.delete("/metrics/{metric_id}", response_model=Metric)
def delete_metric(
    metric_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Delete a metric (admin only).
    """
    if not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to delete this metric."
        )
    metric = metric_service.delete_metric(db, metric_id=metric_id)
    return metric

