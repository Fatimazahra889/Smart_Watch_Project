from pydantic import BaseModel, Field, validator
from typing import Optional, List
from datetime import datetime, timezone
from app.models.enums import MetricType


class MetricBase(BaseModel):
    """Base schema for metric operations."""
    metric_type: MetricType = Field(..., description="Type of metric measurement")
    value: Optional[float] = Field(None, description="Measured value")
    unit: str = Field(..., min_length=1, max_length=20, description="Unit of measurement")
    sensor_model: str = Field(..., min_length=1, max_length=100, description="Sensor model identifier")
    timestamp: Optional[datetime] = Field(None, description="Measurement timestamp")
    user_id: Optional[str] = Field(None, min_length=36, max_length=36, description="User ID associated with the metric")

    @validator('value')
    def validate_value_range(cls, v):
        """Validate value is within reasonable ranges based on metric type."""
        if v is not None:
            if v < 0:
                raise ValueError('Metric value cannot be negative')
        return v

    @validator('timestamp')
    def validate_timestamp(cls, v):
        """Validate timestamp is not in the future."""
        if v and v > datetime.now(timezone.utc):
            raise ValueError('Timestamp cannot be in the future')
        return v


class MetricCreate(MetricBase):
    """Schema for creating individual metrics."""
    pass


class MetricBatchCreate(BaseModel):
    """Schema for batch metric creation."""
    metrics: List[MetricCreate] = Field(..., min_length=1, max_length=1000, description="List of metrics to create")

    @validator('metrics')
    def validate_batch_size(cls, v):
        """Validate batch size limits."""
        if len(v) > 1000:
            raise ValueError('Batch cannot exceed 1000 metrics')
        return v


class MetricResponse(BaseModel):
    """Schema for API responses."""
    id: str
    metric_type: MetricType
    value: Optional[float]
    unit: str
    sensor_model: str
    timestamp: Optional[datetime]
    user_id: str
    created_at: datetime
    updated_at: datetime
    deleted_at: Optional[datetime]

    class Config:
        from_attributes = True


class MetricUpdate(BaseModel):
    """Schema for updating metrics."""
    metric_type: Optional[MetricType] = Field(None, description="Type of metric measurement")
    value: Optional[float] = Field(None, description="Measured value")
    unit: Optional[str] = Field(None, min_length=1, max_length=20, description="Unit of measurement")
    sensor_model: Optional[str] = Field(None, min_length=1, max_length=100, description="Sensor model identifier")
    timestamp: Optional[datetime] = Field(None, description="Measurement timestamp")

    @validator('value')
    def validate_value_range(cls, v):
        """Validate value is within reasonable ranges based on metric type."""
        if v is not None and v < 0:
            raise ValueError('Metric value cannot be negative')
        return v

    @validator('timestamp')
    def validate_timestamp(cls, v):
        """Validate timestamp is not in the future."""
        if v and v > datetime.now(timezone.utc):
            raise ValueError('Timestamp cannot be in the future')
        return v


class MetricSummary(BaseModel):
    """Schema for metric summary responses."""
    metric_type: MetricType
    count: int
    average_value: Optional[float]
    min_value: Optional[float]
    max_value: Optional[float]
    latest_timestamp: Optional[datetime]
    unit: str


class DailyMetricSummary(BaseModel):
    """Schema for daily metric summary responses."""
    date: str
    average_value: Optional[float]
    count: int
    metric_type: MetricType
    unit: str


class DailyMetricsSummaryResponse(BaseModel):
    """Schema for response containing multiple daily summaries."""
    daily_summaries: List[DailyMetricSummary]
    metric_type: MetricType
    start_date: Optional[datetime]
    end_date: Optional[datetime]
    total_days: int
    days_with_data: int