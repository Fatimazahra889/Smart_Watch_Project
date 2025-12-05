from sqlalchemy.orm import Session
from app.models.metric import Metric
from app.schemas.metric import MetricCreate
from app.repositories.base import BaseRepository
from typing import List, Dict, Any
from uuid import UUID
from sqlalchemy import func
from app.models.enums import MetricType

class MetricRepository(BaseRepository[Metric]):
    def create(self, db: Session, *, obj_in: MetricCreate) -> Metric:
        db_obj = Metric(**obj_in.dict())
        db.add(db_obj)
        db.commit()
        db.refresh(db_obj)
        return db_obj

    def create_many(self, db: Session, *, objs_in: List[MetricCreate]) -> List[Metric]:
        db_objs = [Metric(**obj_in.dict()) for obj_in in objs_in]
        db.add_all(db_objs)
        db.commit()
        # We can't refresh multiple objects, so we just return them without the db-generated values
        return db_objs

    def get_summary(
        self, db: Session, *, user_id: UUID, period: str, metric_type: MetricType
    ) -> List[Dict[str, Any]]:
        return (
            db.query(
                func.date_trunc(period, Metric.timestamp).label("period"),
                func.avg(Metric.value).label("value"),
            )
            .filter(Metric.user_id == user_id)
            .filter(Metric.metric_type == metric_type)
            .group_by(func.date_trunc(period, Metric.timestamp))
            .order_by(func.date_trunc(period, Metric.timestamp))
            .all()
        )

    def get_by_user_and_type(
        self, db: Session, *, user_id: UUID, metric_type: MetricType
    ) -> List[Metric]:
        return (
            db.query(Metric)
            .filter(Metric.user_id == user_id, Metric.metric_type == metric_type)
            .order_by(Metric.timestamp.desc())
            .all()
        )

    def get_multi(
        self, db: Session, *, skip: int = 0, limit: int = 100
    ) -> List[Metric]:
        return db.query(self.model).offset(skip).limit(limit).all()

    def remove(self, db: Session, *, id: str) -> Metric | None:
        obj = db.query(self.model).get(id)
        if obj:
            db.delete(obj)
            db.commit()
        return obj

metric_repository = MetricRepository(Metric)
