from __future__ import annotations
from pydantic import BaseModel, ConfigDict
from typing import Optional, List
import uuid
from datetime import datetime
from app.models.enums import MetricType

# Shared properties
class MetricBase(BaseModel):
    metric_type: MetricType
    value: Optional[float] = None
    unit: Optional[str] = None
    sensor_model: Optional[str] = None
    timestamp: datetime


# Properties to receive via API on creation
class MetricCreate(MetricBase):
    device_id: uuid.UUID
    user_id: uuid.UUID

# Properties shared by models in DB
class MetricInDBBase(MetricBase):
    id: uuid.UUID
    device_id: uuid.UUID
    user_id: uuid.UUID
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)

# Properties to return to client
class Metric(MetricInDBBase):
    pass

class MetricBatch(BaseModel):
    metrics: List[MetricBase]
