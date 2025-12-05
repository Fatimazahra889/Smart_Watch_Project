from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.api import deps
from app.schemas.issue import Issue, IssueCreate, IssueUpdate
from app.services.issue_service import issue_service
from app.models.user import User
from uuid import UUID

router = APIRouter()

@router.post("/issues/", response_model=Issue)
def create_issue(
    *,
    db: Session = Depends(deps.get_db),
    issue_in: IssueCreate,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Create new issue.
    """
    issue = issue_service.create_issue(db, issue_in=issue_in, user_id=current_user.id)
    return issue

@router.get("/issues/", response_model=list[Issue])
def read_issues(
    db: Session = Depends(deps.get_db),
    skip: int = 0,
    limit: int = 100,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Retrieve issues.
    """
    issues = issue_service.get_multi(db, skip=skip, limit=limit)
    return issues

@router.get("/issues/{issue_id}", response_model=Issue)
def read_issue_by_id(
    issue_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_user),
):
    """
    Get a specific issue by ID.
    """
    issue = issue_service.get(db, id=str(issue_id))
    if issue.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this issue."
        )
    return issue

@router.put("/issues/{issue_id}", response_model=Issue)
def update_issue(
    *,
    db: Session = Depends(deps.get_db),
    issue_id: UUID,
    issue_in: IssueUpdate,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Update an issue.
    """
    issue = issue_service.get(db, id=str(issue_id))
    if issue.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to update this issue."
        )
    issue = issue_service.update_issue(db, db_obj=issue, obj_in=issue_in)
    return issue

@router.delete("/issues/{issue_id}", response_model=Issue)
def delete_issue(
    *,
    db: Session = Depends(deps.get_db),
    issue_id: UUID,
    current_user: User = Depends(deps.get_current_user),
):
    """
    Delete an issue (soft delete).
    """
    issue = issue_service.get(db, id=str(issue_id))
    if issue.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to delete this issue."
        )
    issue = issue_service.delete_issue(db, id=str(issue_id))
    return issue
