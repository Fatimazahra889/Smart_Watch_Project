from sqlalchemy.orm import Session
from app.models.issue import Issue
from app.schemas.issue import IssueCreate, IssueUpdate
from app.repositories.issue_repository import issue_repository
from app.core.exceptions import IssueNotFoundException
from typing import List, Dict, Any
from uuid import UUID

class IssueService:
    def __init__(self, issue_repo):
        self.issue_repo = issue_repo

    def get(self, db: Session, id: str) -> Issue | None:
        issue = self.issue_repo.get(db, id=id)
        if not issue:
            raise IssueNotFoundException(issue_id=id)
        return issue

    def get_multi(
        self, db: Session, *, skip: int = 0, limit: int = 100
    ) -> List[Issue]:
        return self.issue_repo.get_multi(db, skip=skip, limit=limit)

    def create_issue(self, db: Session, *, issue_in: IssueCreate, user_id: UUID) -> Issue:
        issue_data = issue_in.model_dump()
        issue_data["user_id"] = user_id
        issue = self.issue_repo.create(db, obj_in=issue_data)
        return issue

    def update_issue(
        self, db: Session, *, db_obj: Issue, obj_in: IssueUpdate | Dict[str, Any]
    ) -> Issue:
        return self.issue_repo.update(db, db_obj=db_obj, obj_in=obj_in)

    def delete_issue(self, db: Session, *, id: str) -> Issue | None:
        issue = self.issue_repo.remove(db, id=id)
        if not issue:
            raise IssueNotFoundException(issue_id=id)
        return issue

issue_service = IssueService(issue_repository)
