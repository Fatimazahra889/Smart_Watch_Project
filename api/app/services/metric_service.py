from sqlalchemy.orm import Session
from app.models.device import Device
from app.schemas.metric import MetricBatch, MetricCreate
from app.repositories.metric_repository import metric_repository
from typing import Any, List
from app.core.exceptions import MetricCreationException, MetricNotFoundException
from uuid import UUID

class MetricService:
    def create_metrics(self, db: Session, *, device: Device, metrics_in: MetricBatch) -> List[dict]:
        metrics_to_create = [
            MetricCreate(**metric.dict(), device_id=device.id, user_id=device.user_id)
            for metric in metrics_in.metrics
        ]
        try:
            created_metrics = metric_repository.create_many(db, objs_in=metrics_to_create)
            return [{"status": "ok"} for _ in created_metrics]
        except Exception as e:
            # In a real app, you would log the error here
            print(f"Error creating metrics: {e}")
            raise MetricCreationException()

    def get_metric(self, db: Session, *, metric_id: UUID) -> Any:
        metric = metric_repository.get(db, id=metric_id)
        if not metric:
            raise MetricNotFoundException(metric_id=metric_id)
        return metric

    def get_metrics(
        self, db: Session, *, skip: int = 0, limit: int = 100
    ) -> List[Any]:
        return metric_repository.get_multi(db, skip=skip, limit=limit)

    def delete_metric(self, db: Session, *, metric_id: UUID) -> Any:
        metric = metric_repository.get(db, id=metric_id)
        if not metric:
            raise MetricNotFoundException(metric_id=metric_id)
        return metric_repository.remove(db, id=metric_id)

metric_service = MetricService()
