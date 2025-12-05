from __future__ import annotations
from pydantic import BaseModel, ConfigDict
from typing import Optional, List
import uuid
from datetime import datetime
from .metric import Metric

# Shared properties
class DeviceBase(BaseModel):
    name: Optional[str] = None
    serial_number: Optional[str] = None
    model: Optional[str] = None
    firmware_version: Optional[str] = None
    is_active: Optional[bool] = True
    registered_at: Optional[datetime] = None

# Properties to receive via API on registration
class DeviceRegister(BaseModel):
    serial_number: str
    name: Optional[str] = None

# Properties to receive via API on creation
class DeviceCreate(BaseModel):
    name: Optional[str] = None
    serial_number: str
    api_key: str
    user_id: uuid.UUID
    registered_at: datetime

# Properties to receive via API on update
class DeviceUpdate(DeviceBase):
    pass

# Properties shared by models in DB
class DeviceInDBBase(DeviceBase):
    id: uuid.UUID
    user_id: uuid.UUID
    api_key: str
    created_at: datetime
    updated_at: datetime
    deleted_at: Optional[datetime] = None

    model_config = ConfigDict(from_attributes=True)

# Properties to return to client
class Device(DeviceInDBBase):
    pass

class DeviceWithMetrics(Device):
    metrics: List[Metric] = []

# Properties to return to client on registration
class DeviceRegistrationResponse(BaseModel):
    api_key: str
    device: Device