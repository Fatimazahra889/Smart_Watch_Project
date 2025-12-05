from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.api import deps
from app.models.user import User
from app.schemas.device import Device, DeviceRegister, DeviceRegistrationResponse, DeviceUpdate
from app.services.device_service import device_service
from typing import List
from uuid import UUID
from app.schemas.metric import Metric

router = APIRouter()

@router.post("/devices/register", response_model=DeviceRegistrationResponse)
def register_device(
    *,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
    device_in: DeviceRegister,
) -> dict:
    """
    Register a new device.
    """
    result = device_service.register_device(db, user=current_user, device_in=device_in)
    return result

@router.get("/devices/", response_model=List[Device])
def get_devices(
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
    skip: int = 0,
    limit: int = 100,
):
    """
    Retrieve devices for the current user.
    """
    devices = device_service.get_devices(
        db, user_id=current_user.id, skip=skip, limit=limit
    )
    return devices

@router.get("/devices/{device_id}", response_model=Device)
def get_device(
    device_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Get a specific device by ID.
    """
    device = device_service.get_device(db, device_id=device_id)
    if device.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this device."
        )
    return device

@router.put("/devices/{device_id}", response_model=Device)
def update_device(
    device_id: UUID,
    device_in: DeviceUpdate,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Update a device.
    """
    device = device_service.get_device(db, device_id=device_id)
    if device.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to update this device."
        )
    device = device_service.update_device(db, device_id=device_id, obj_in=device_in)
    return device

@router.delete("/devices/{device_id}", response_model=Device)
def delete_device(
    device_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Delete a device.
    """
    device = device_service.get_device(db, device_id=device_id)
    if device.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to delete this device."
        )
    device = device_service.delete_device(db, device_id=device_id)
    return device

@router.get("/devices/{device_id}/metrics", response_model=List[Metric])
def get_device_metrics(
    device_id: UUID,
    db: Session = Depends(deps.get_db),
    current_user: User = Depends(deps.get_current_active_user),
):
    """
    Get all metrics for a specific device.
    """
    device = device_service.get_device(db, device_id=device_id)
    if device.user_id != current_user.id and not current_user.is_admin:
        raise HTTPException(
            status_code=403, detail="You do not have permission to access this device's metrics."
        )
    return device.metrics

