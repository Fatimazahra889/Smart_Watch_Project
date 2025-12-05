from pydantic import BaseModel
from datetime import datetime
from typing import List

class MetricSummary(BaseModel):
    period: datetime
    value: float

class MetricsSummaryResponse(BaseModel):
    metrics: List[MetricSummary]
