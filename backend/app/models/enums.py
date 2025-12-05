import enum

class MetricType(str, enum.Enum):
    SPO2 = "spo2"
    HEART_RATE = "heart_rate"
    SKIN_TEMPERATURE = "skin_temperature"
    AMBIENT_TEMPERATURE = "ambient_temperature"

class IssueSeverity(str, enum.Enum):
    LOW = "low"
    MODERATE = "moderate"
    CRITICAL = "critical"

class AggregationPeriod(str, enum.Enum):
    DAILY = "day"
    WEEKLY = "week"
    MONTHLY = "month"
