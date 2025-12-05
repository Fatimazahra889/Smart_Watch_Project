from sqlalchemy.orm import Session
from app.models.issue import Issue
from app.schemas.issue import IssueCreate, IssueUpdate
from app.repositories.base import BaseRepository
from typing import TypeVar, Generic, List, Dict, Any
from datetime import datetime

ModelType = TypeVar("ModelType")

class IssueRepository(BaseRepository[Issue]):
    def create(self, db: Session, *, obj_in: Dict[str, Any]) -> Issue:
        db_obj = Issue(**obj_in)
        db.add(db_obj)
        db.commit()
        db.refresh(db_obj)
        return db_obj

    def get_multi(
        self, db: Session, *, skip: int = 0, limit: int = 100
    ) -> List[Issue]:
        return db.query(self.model).offset(skip).limit(limit).all()

    def update(
        self, db: Session, *, db_obj: Issue, obj_in: IssueUpdate | Dict[str, Any]
    ) -> Issue:
        if isinstance(obj_in, dict):
            update_data = obj_in
        else:
            update_data = obj_in.model_dump(exclude_unset=True)
        
        for field in update_data:
            if hasattr(db_obj, field):
                setattr(db_obj, field, update_data[field])
        
        db.add(db_obj)
        db.commit()
        db.refresh(db_obj)
        return db_obj

    def remove(self, db: Session, *, id: str) -> Issue | None:
        obj = db.query(self.model).get(id)
        if obj:
            obj.deleted_at = datetime.utcnow()
            db.add(obj)
            db.commit()
            db.refresh(obj)
        return obj

issue_repository = IssueRepository(Issue)
