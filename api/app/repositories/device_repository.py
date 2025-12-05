from sqlalchemy.orm import Session
from app.models.device import Device
from app.schemas.device import DeviceCreate, DeviceUpdate
from app.repositories.base import BaseRepository

from typing import List, Dict, Any
from uuid import UUID
from datetime import datetime

class DeviceRepository(BaseRepository[Device]):
    def get_by_api_key(self, db: Session, *, api_key: str) -> Device | None:
        return db.query(Device).filter(Device.api_key == api_key).first()

    def get_by_serial_number(self, db: Session, *, serial_number: str) -> Device | None:
        return db.query(Device).filter(Device.serial_number == serial_number).first()

    def create(self, db: Session, *, obj_in: DeviceCreate) -> Device:
        db_obj = Device(
            name=obj_in.name,
            serial_number=obj_in.serial_number,
            api_key=obj_in.api_key,
            user_id=obj_in.user_id,
            registered_at=obj_in.registered_at
        )
        db.add(db_obj)
        db.commit()
        db.refresh(db_obj)
        return db_obj

    def get_multi_by_owner(
        self, db: Session, *, user_id: UUID, skip: int = 0, limit: int = 100
    ) -> List[Device]:
        return (
            db.query(self.model)
            .filter(Device.user_id == user_id)
            .offset(skip)
            .limit(limit)
            .all()
        )

    def update(
        self, db: Session, *, db_obj: Device, obj_in: DeviceUpdate | Dict[str, Any]
    ) -> Device:
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

    def remove(self, db: Session, *, id: str) -> Device | None:
        obj = db.query(self.model).get(id)
        if obj:
            obj.deleted_at = datetime.utcnow()
            db.add(obj)
            db.commit()
            db.refresh(obj)
        return obj


device_repository = DeviceRepository(Device)
