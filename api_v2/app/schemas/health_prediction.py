from pydantic import BaseModel, Field
from typing import Optional, Dict, List
from datetime import datetime
from enum import Enum


class HealthRiskLevel(str, Enum):
    """Enum for health risk levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class HealthPredictionRequest(BaseModel):
    """Schema for health prediction requests."""
    user_id: str = Field(..., description="User ID for prediction")
    spo2: Optional[float] = Field(None, ge=0.0, le=100.0, description="SpO2 value in percentage")
    heart_rate: Optional[float] = Field(None, ge=0.0, description="Heart rate in beats per minute")
    body_temperature: Optional[float] = Field(None, ge=0.0, description="Body temperature in Celsius")
    include_metrics: bool = Field(False, description="Include raw metrics in response")
    prediction_horizon_hours: Optional[int] = Field(
        24,
        ge=1,
        le=168,
        description="Prediction horizon in hours (1-168)"
    )


class MetricAverage(BaseModel):
    """Schema for metric averages."""
    metric_type: str = Field(..., description="Type of metric")
    average_value: Optional[float] = Field(None, description="Average value over period")
    unit: str = Field(..., description="Unit of measurement")
    data_points: int = Field(..., description="Number of data points used")
    is_healthy: bool = Field(..., description="Whether metric is in healthy range")
    health_score: float = Field(..., ge=0.0, le=1.0, description="Health score for this metric (0-1)")


class HealthPredictionResponse(BaseModel):
    """Schema for health prediction responses."""
    user_id: str = Field(..., description="User ID")
    prediction_timestamp: datetime = Field(..., description="When prediction was made")
    prediction_result: int = Field(..., ge=0, le=2, description="Prediction result: 0=normal, 1=sick, 2=life-threatening")
    health_risk_level: HealthRiskLevel = Field(..., description="Overall health risk level")
    confidence_score: float = Field(..., ge=0.0, le=1.0, description="Prediction confidence (0-1)")
    
    # Detailed metrics analysis
    metric_averages: List[MetricAverage] = Field(..., description="Average values for each metric type")
    
    # Risk factors and recommendations
    risk_factors: List[str] = Field(default_factory=list, description="Identified risk factors")
    recommendations: List[str] = Field(default_factory=list, description="Health recommendations")
    
    # Raw metrics (optional)
    raw_metrics_summary: Optional[Dict] = Field(None, description="Summary of raw metrics used")
    
    # Model information
    model_version: str = Field(..., description="Prediction model version")
    prediction_horizon_hours: int = Field(..., description="Prediction horizon used")


class HealthTrend(BaseModel):
    """Schema for health trend analysis."""
    period_start: datetime = Field(..., description="Start of analysis period")
    period_end: datetime = Field(..., description="End of analysis period")
    health_score: float = Field(..., ge=0.0, le=1.0, description="Health score for period")
    trend_direction: str = Field(..., description="Trend direction (improving, stable, declining)")
    trend_strength: float = Field(..., ge=0.0, le=1.0, description="Strength of trend (0-1)")


class HealthHistoryResponse(BaseModel):
    """Schema for health history responses."""
    user_id: str = Field(..., description="User ID")
    current_prediction: HealthPredictionResponse = Field(..., description="Current health prediction")
    health_trends: List[HealthTrend] = Field(..., description="Historical health trends")
    period_days: int = Field(..., description="Number of days in history")