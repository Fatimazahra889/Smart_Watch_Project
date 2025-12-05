from sqlalchemy.orm import Session
from app.models.user import User
from app.schemas.device import DeviceRegister, DeviceCreate
from app.repositories.device_repository import device_repository
from app.core.security import generate_api_key
from datetime import datetime
from app.core.exceptions import DeviceAlreadyExistsException

from app.schemas.device import DeviceRegister, DeviceCreate, DeviceUpdate
from app.repositories.device_repository import device_repository
from app.core.security import generate_api_key
from datetime import datetime
from app.core.exceptions import DeviceAlreadyExistsException, DeviceNotFoundException
from typing import List, Dict, Any
from uuid import UUID

class DeviceService:
    def register_device(self, db: Session, *, user: User, device_in: DeviceRegister) -> dict:
        existing_device = device_repository.get_by_serial_number(db, serial_number=device_in.serial_number)
        if existing_device:
            raise DeviceAlreadyExistsException(serial_number=device_in.serial_number)

        api_key = generate_api_key()
        device_create = DeviceCreate(
            name=device_in.name,
            serial_number=device_in.serial_number,
            api_key=api_key,
            user_id=user.id,
            registered_at=datetime.utcnow()
        )
        device = device_repository.create(db, obj_in=device_create)
        return {"api_key": api_key, "device": device}

    def get_device(self, db: Session, *, device_id: UUID) -> Any:
        device = device_repository.get(db, id=device_id)
        if not device:
            raise DeviceNotFoundException(device_id=device_id)
        return device

    def get_devices(
        self, db: Session, *, user_id: UUID, skip: int = 0, limit: int = 100
    ) -> List[Any]:
        return device_repository.get_multi_by_owner(
            db, user_id=user_id, skip=skip, limit=limit
        )

    def update_device(
        self, db: Session, *, device_id: UUID, obj_in: DeviceUpdate
    ) -> Any:
        device = device_repository.get(db, id=device_id)
        if not device:
            raise DeviceNotFoundException(device_id=device_id)
        return device_repository.update(db, db_obj=device, obj_in=obj_in)

    def delete_device(self, db: Session, *, device_id: UUID) -> Any:
        device = device_repository.get(db, id=device_id)
        if not device:
            raise DeviceNotFoundException(device_id=device_id)
        return device_repository.remove(db, id=device_id)

device_service = DeviceService()
