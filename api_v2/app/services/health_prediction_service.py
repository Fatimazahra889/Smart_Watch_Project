from datetime import datetime, timedelta, timezone
from typing import Optional, List, Tuple
from sqlalchemy.orm import Session
import logging
import pickle
import numpy as np
import os

from app.models.enums import MetricType
from app.repositories.metric_repository import MetricRepository
from app.schemas.health_prediction import (
    HealthPredictionRequest,
    HealthPredictionResponse,
    HealthRiskLevel,
    MetricAverage
)
from app.core.exceptions import UserNotFoundException

# Configure logging
logger = logging.getLogger(__name__)


class HealthPredictionService:
    """Service for health status prediction based on SpO2, heart rate, and body temperature metrics."""
    
    def __init__(self, db: Session):
        self.db = db
        self.metric_repo = MetricRepository(db)
        
        # Load ML model
        self.ml_model = self._load_ml_model()
        
        # Health parameter thresholds for prediction logic
        self.health_thresholds = {
            # SpO2 thresholds (normal: 95-100%, concerning: 90-94%, critical: <90%)
            'spo2_normal_min': 95.0,
            'spo2_concerning_min': 90.0,
            
            # Heart rate thresholds (normal: 60-100 bpm, concerning: 40-59 or 101-120, critical: <40 or >120)
            'heart_rate_normal_min': 60.0,
            'heart_rate_normal_max': 100.0,
            'heart_rate_concerning_min': 40.0,
            'heart_rate_concerning_max': 120.0,
            
            # Body temperature thresholds (normal: 36.1-37.2°C, concerning: 37.3-38.0 or 35.0-36.0, critical: >38.0 or <35.0)
            'temp_normal_min': 36.1,
            'temp_normal_max': 37.2,
            'temp_concerning_min': 35.0,
            'temp_concerning_max': 38.0,
        }

    def _load_ml_model(self):
        """
        Load the ML model from the pickle file.
        
        Returns:
            Loaded ML model or None if model is not available
        """
        try:
            model_path = os.path.join(os.path.dirname(__file__), '..', 'ml', 'model.pkl')
            
            with open(model_path, 'rb') as model_file:
                model = pickle.load(model_file)
                logger.info("ML model loaded successfully")
            return model
        except Exception as e:
            logger.warning(f"ML model not available, using rule-based prediction: {e}")
            return None

    def _get_health_parameters(self, request: HealthPredictionRequest) -> tuple:
        """
        Get health parameters from request or database.
        
        Args:
            request: Health prediction request
            
        Returns:
            Tuple of (spo2, heart_rate, body_temperature) values
        """
        spo2 = request.spo2
        heart_rate = request.heart_rate
        body_temperature = request.body_temperature
        
        # If parameters not provided in request, try to get from database
        if spo2 is None:
            spo2 = self._get_metric_from_db(request.user_id, MetricType.SPO2)
        
        if heart_rate is None:
            heart_rate = self._get_metric_from_db(request.user_id, MetricType.HEART_RATE)
        
        if body_temperature is None:
            body_temperature = self._get_metric_from_db(request.user_id, MetricType.SKIN_TEMPERATURE)
        
        return spo2, heart_rate, body_temperature

    def _get_metric_from_db(self, user_id: str, metric_type: MetricType) -> Optional[float]:
        """
        Get metric value from the database for the last 24 hours.
        
        Args:
            user_id: User ID to get data for
            metric_type: Type of metric to retrieve
            
        Returns:
            Average metric value or None if no data
        """
        try:
            end_time = datetime.now(timezone.utc)
            start_time = end_time - timedelta(hours=24)
            
            # Get metrics for the last 24 hours
            metrics = self.metric_repo.get_user_metrics(
                user_id=user_id,
                metric_type=metric_type,
                start_date=start_time,
                end_date=end_time
            )
            
            # Filter metrics with values and calculate average
            metrics_with_values = [m for m in metrics if m.value is not None]
            
            if metrics_with_values:
                values = [float(str(m.value)) for m in metrics_with_values]
                average_value = sum(values) / len(values)
                logger.info(f"Found {len(values)} {metric_type.value} data points, average: {average_value}")
                return average_value
            else:
                logger.warning(f"No {metric_type.value} data found for user {user_id} in last 24 hours")
                return None
                
        except Exception as e:
            logger.error(f"Failed to get {metric_type.value} data for user {user_id}: {e}")
            return None

    def _predict_health_status(self, spo2: Optional[float], heart_rate: Optional[float], body_temperature: Optional[float]) -> int:
        """
        Predict health status based on SpO2, heart rate, and body temperature.
        
        Args:
            spo2: SpO2 value in percentage
            heart_rate: Heart rate in beats per minute
            body_temperature: Body temperature in Celsius
            
        Returns:
            Prediction result: 0=normal, 1=sick, 2=life-threatening
        """
        critical_count = 0
        concerning_count = 0
        
        # Check SpO2
        if spo2 is not None:
            if spo2 < self.health_thresholds['spo2_concerning_min']:
                critical_count += 1
            elif spo2 < self.health_thresholds['spo2_normal_min']:
                concerning_count += 1
        
        # Check heart rate
        if heart_rate is not None:
            if (heart_rate < self.health_thresholds['heart_rate_concerning_min'] or
                heart_rate > self.health_thresholds['heart_rate_concerning_max']):
                critical_count += 1
            elif (heart_rate < self.health_thresholds['heart_rate_normal_min'] or
                  heart_rate > self.health_thresholds['heart_rate_normal_max']):
                concerning_count += 1
        
        # Check body temperature
        if body_temperature is not None:
            if (body_temperature < self.health_thresholds['temp_concerning_min'] or
                body_temperature > self.health_thresholds['temp_concerning_max']):
                critical_count += 1
            elif (body_temperature < self.health_thresholds['temp_normal_min'] or
                  body_temperature > self.health_thresholds['temp_normal_max']):
                concerning_count += 1
        
        # Determine prediction result
        if critical_count > 0:
            return 2  # Life-threatening condition
        elif concerning_count > 0:
            return 1  # Sick but not life-threatening
        else:
            return 0  # Normal health

    def _map_prediction_to_risk_level(self, prediction_result: int) -> HealthRiskLevel:
        """
        Map prediction result to health risk level.
        
        Args:
            prediction_result: Prediction result (0, 1, or 2)
            
        Returns:
            Corresponding HealthRiskLevel
        """
        if prediction_result == 0:
            return HealthRiskLevel.LOW
        elif prediction_result == 1:
            return HealthRiskLevel.MEDIUM
        else:  # prediction_result == 2
            return HealthRiskLevel.HIGH

    def predict_health_status(self, request: HealthPredictionRequest) -> HealthPredictionResponse:
        """
        Predict health status based on SpO2, heart rate, and body temperature metrics.
        
        Args:
            request: Health prediction request
            
        Returns:
            Health prediction response with prediction result and risk level
        """
        logger.info(f"Starting health prediction for user {request.user_id}")
        
        try:
            # Get health parameters from request or database
            spo2, heart_rate, body_temperature = self._get_health_parameters(request)
            
            # Make prediction using the new logic
            prediction_result = self._predict_health_status(spo2, heart_rate, body_temperature)
            
            # Map prediction result to health risk level
            health_risk_level = self._map_prediction_to_risk_level(prediction_result)
            
            # Calculate confidence score based on available data
            available_params = sum(1 for param in [spo2, heart_rate, body_temperature] if param is not None)
            confidence_score = available_params / 3.0
            
            # Generate metric averages for response
            metric_averages = self._generate_metric_averages(spo2, heart_rate, body_temperature)
            
            # Generate risk factors and recommendations
            risk_factors, recommendations = self._generate_risk_analysis(prediction_result, spo2, heart_rate, body_temperature)
            
            logger.info(f"Prediction completed: result={prediction_result}, risk={health_risk_level.value}, confidence={confidence_score:.2f}")

            response = HealthPredictionResponse(
                user_id=request.user_id,
                prediction_timestamp=datetime.now(timezone.utc),
                prediction_result=prediction_result,
                health_risk_level=health_risk_level,
                confidence_score=confidence_score,
                metric_averages=metric_averages,
                risk_factors=risk_factors,
                recommendations=recommendations,
                raw_metrics_summary=None,
                model_version="v3.0.0-multi-parameter",
                prediction_horizon_hours=request.prediction_horizon_hours or 24
            )
            
            return response
            
        except UserNotFoundException:
            logger.error(f"User not found for health prediction: {request.user_id}")
            raise
        except Exception as e:
            logger.error(f"Health prediction failed for user {request.user_id}: {e}")
            raise

    def _generate_metric_averages(self, spo2: Optional[float], heart_rate: Optional[float], body_temperature: Optional[float]) -> List[MetricAverage]:
        """
        Generate metric averages for the response.
        
        Args:
            spo2: SpO2 value
            heart_rate: Heart rate value
            body_temperature: Body temperature value
            
        Returns:
            List of MetricAverage objects
        """
        metric_averages = []
        
        if spo2 is not None:
            is_healthy = spo2 >= self.health_thresholds['spo2_normal_min']
            health_score = 1.0 if is_healthy else 0.5 if spo2 >= self.health_thresholds['spo2_concerning_min'] else 0.0
            metric_averages.append(MetricAverage(
                metric_type="spo2",
                average_value=spo2,
                unit="%",
                data_points=1,
                is_healthy=is_healthy,
                health_score=health_score
            ))
        
        if heart_rate is not None:
            is_healthy = (self.health_thresholds['heart_rate_normal_min'] <= heart_rate <= self.health_thresholds['heart_rate_normal_max'])
            health_score = 1.0 if is_healthy else 0.5 if (self.health_thresholds['heart_rate_concerning_min'] <= heart_rate <= self.health_thresholds['heart_rate_concerning_max']) else 0.0
            metric_averages.append(MetricAverage(
                metric_type="heart_rate",
                average_value=heart_rate,
                unit="bpm",
                data_points=1,
                is_healthy=is_healthy,
                health_score=health_score
            ))
        
        if body_temperature is not None:
            is_healthy = (self.health_thresholds['temp_normal_min'] <= body_temperature <= self.health_thresholds['temp_normal_max'])
            health_score = 1.0 if is_healthy else 0.5 if (self.health_thresholds['temp_concerning_min'] <= body_temperature <= self.health_thresholds['temp_concerning_max']) else 0.0
            metric_averages.append(MetricAverage(
                metric_type="body_temperature",
                average_value=body_temperature,
                unit="°C",
                data_points=1,
                is_healthy=is_healthy,
                health_score=health_score
            ))
        
        return metric_averages

    def _generate_risk_analysis(self, prediction_result: int, spo2: Optional[float], heart_rate: Optional[float], body_temperature: Optional[float]) -> Tuple[List[str], List[str]]:
        """
        Generate risk factors and recommendations based on prediction result.
        
        Args:
            prediction_result: Prediction result (0, 1, or 2)
            spo2: SpO2 value
            heart_rate: Heart rate value
            body_temperature: Body temperature value
            
        Returns:
            Tuple of (risk_factors, recommendations)
        """
        risk_factors = []
        recommendations = []
        
        if prediction_result == 0:
            recommendations.append("All health parameters are within normal ranges. Continue monitoring.")
        else:
            # Add specific risk factors
            if spo2 is not None and spo2 < self.health_thresholds['spo2_normal_min']:
                risk_factors.append(f"Low SpO2 level ({spo2}%)")
                if spo2 < self.health_thresholds['spo2_concerning_min']:
                    recommendations.append("Seek immediate medical attention for low oxygen levels")
                else:
                    recommendations.append("Monitor oxygen levels and rest")
            
            if heart_rate is not None:
                if heart_rate < self.health_thresholds['heart_rate_normal_min']:
                    risk_factors.append(f"Low heart rate ({heart_rate} bpm)")
                    if heart_rate < self.health_thresholds['heart_rate_concerning_min']:
                        recommendations.append("Seek immediate medical attention for bradycardia")
                    else:
                        recommendations.append("Monitor heart rate and avoid strenuous activity")
                elif heart_rate > self.health_thresholds['heart_rate_normal_max']:
                    risk_factors.append(f"High heart rate ({heart_rate} bpm)")
                    if heart_rate > self.health_thresholds['heart_rate_concerning_max']:
                        recommendations.append("Seek immediate medical attention for tachycardia")
                    else:
                        recommendations.append("Rest and monitor heart rate")
            
            if body_temperature is not None:
                if body_temperature < self.health_thresholds['temp_normal_min']:
                    risk_factors.append(f"Low body temperature ({body_temperature}°C)")
                    if body_temperature < self.health_thresholds['temp_concerning_min']:
                        recommendations.append("Seek immediate medical attention for hypothermia")
                    else:
                        recommendations.append("Warm up and monitor temperature")
                elif body_temperature > self.health_thresholds['temp_normal_max']:
                    risk_factors.append(f"High body temperature ({body_temperature}°C)")
                    if body_temperature > self.health_thresholds['temp_concerning_max']:
                        recommendations.append("Seek immediate medical attention for fever")
                    else:
                        recommendations.append("Rest, hydrate, and monitor temperature")
        
        return risk_factors, recommendations