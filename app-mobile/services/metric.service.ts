import { ApiError, apiService, NetworkError, ValidationError } from './api';
import { offlineStorageService } from './offline-storage.service';

export enum MetricType {
  SPO2 = "spo2",
  HEART_RATE = "heart_rate",
  SKIN_TEMPERATURE = "skin_temperature",
  AMBIENT_TEMPERATURE = "ambient_temperature",
  STEPS = "steps",
  SLEEP = "sleep"
}

export interface MetricCreate {
  metric_type: MetricType;
  value?: number;
  unit: string;
  sensor_model: string;
  timestamp?: string;
  user_id?: string;
}

export interface MetricBatchCreate {
  metrics: MetricCreate[];
}

export interface MetricResponse {
  id: string;
  metric_type: MetricType;
  value?: number;
  unit: string;
  sensor_model: string;
  timestamp?: string;
  user_id: string;
  created_at: string;
  updated_at: string;
  deleted_at?: string;
}

export interface MetricUpdate {
  value?: number;
  unit?: string;
  sensor_model?: string;
  timestamp?: string;
}

export interface MetricSummary {
  metric_type: MetricType;
  count: number;
  average_value?: number;
  min_value?: number;
  max_value?: number;
  latest_timestamp?: string;
  unit: string;
}

export interface BatchCreateResponse {
  total_processed: number;
  successful: number;
  failed: number;
  created_metrics: MetricResponse[];
  errors?: Array<{
    index: number;
    error: string;
  }>;
  status?: string;
}

export interface MetricsStatistics {
  total_metrics: number;
  metrics_by_type: Array<{
    metric_type: MetricType;
    count: number;
    average_value?: number;
  }>;
  time_range: {
    start_date: string;
    end_date: string;
  };
}

export interface HealthPredictionResponse {
  user_id: string;
  prediction_timestamp: string;
  prediction_result: number;
  health_risk_level: 'low' | 'medium' | 'high';
  confidence_score: number;
  metric_averages: Array<{
    metric_type: string;
    average_value?: number;
    unit: string;
    data_points: number;
    is_healthy: boolean;
    health_score: number;
  }>;
  risk_factors: string[];
  recommendations: string[];
  raw_metrics_summary?: any;
  model_version: string;
  prediction_horizon_hours: number;
}

export interface MetricQueryParams {
  skip?: number;
  limit?: number;
  metric_type?: MetricType;
  start_date?: string;
  end_date?: string;
  user_id?: string;
  sensor_model?: string;
}

export class MetricService {
  private metricsCache: Map<string, MetricResponse[]> = new Map();

  /**
   * Create metrics batch for authenticated user
   * Uses JWT Bearer authentication for all endpoints
   */
  async createMetricsBatch(metricsBatch: MetricBatchCreate): Promise<BatchCreateResponse> {
    // Validate metrics before processing with detailed error reporting
    const validation = this.validateMetricsBatch(metricsBatch);
    if (!validation.valid) {
      console.warn('Metrics batch validation failed:', validation.errors);
      
      // Convert validation errors to the expected format for ValidationError
      const validationDetails = validation.errors.map((error, index) => ({
        loc: ['metrics', index],
        msg: error,
        type: 'value_error'
      }));
      
      throw new ValidationError('Invalid metrics batch', validationDetails);
    }

    try {
      // Check if we should use offline storage
      if (!offlineStorageService.isOnline()) {
        await offlineStorageService.storeMetrics(metricsBatch.metrics);
        return {
          total_processed: metricsBatch.metrics.length,
          successful: 0,
          failed: 0,
          created_metrics: [],
          status: 'queued'
        };
      }

      const response = await apiService.post<any>('/api/v1/metrics/batch', metricsBatch);
      console.log('Metrics batch created successfully:', response);
      // Clear cache after creating new metrics
      this.clearCache();
      
      return response;
    } catch (error) {
      // On network error, store offline and retry later
      if (error instanceof NetworkError || (error instanceof ApiError && error.status >= 500)) {
        await offlineStorageService.storeMetrics(metricsBatch.metrics);
        return {
          total_processed: metricsBatch.metrics.length,
          successful: 0,
          failed: 0,
          created_metrics: [],
          status: 'queued'
        };
      }
      
      // Re-throw validation and authentication errors
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('Metrics batch validation failed', error.details?.detail);
      }
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required for metrics submission', 401, 'UNAUTHORIZED');
      }
      throw error;
    }
  }

  /**
   * Create a single metric
   */
  async createMetric(metric: MetricCreate): Promise<BatchCreateResponse> {
    return this.createMetricsBatch({ metrics: [metric] });
  }

  /**
   * Get metrics with filtering options
   */
  async getMetrics(params: MetricQueryParams = {}): Promise<MetricResponse[]> {
    const cacheKey = this.generateCacheKey(params);
    
    // Check cache first
    if (this.metricsCache.has(cacheKey)) {
      return this.metricsCache.get(cacheKey)!;
    }

    try {
      const queryParams = new URLSearchParams();
      
      if (params.skip) queryParams.append('skip', params.skip.toString());
      if (params.limit) queryParams.append('limit', params.limit.toString());
      if (params.metric_type) queryParams.append('metric_type', params.metric_type);
      if (params.start_date) queryParams.append('start_date', params.start_date);
      if (params.end_date) queryParams.append('end_date', params.end_date);
      if (params.user_id) queryParams.append('user_id', params.user_id);
      if (params.sensor_model) queryParams.append('sensor_model', params.sensor_model);

      const queryString = queryParams.toString();
      const endpoint = queryString ? `/api/v1/metrics/?${queryString}` : '/api/v1/metrics/';
      
      const response = await apiService.get<MetricResponse[]>(endpoint);
      
      // Cache the results
      this.cacheMetrics(cacheKey, response);
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access metrics', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Admin access required to view all metrics', 403, 'FORBIDDEN');
      }
      throw error;
    }
  }

  /**
   * Get metric by ID
   */
  async getMetricById(metricId: string): Promise<MetricResponse> {
    try {
      const response = await apiService.get<MetricResponse>(`/api/v1/metrics/${metricId}`);
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access metric', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to access metric', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Metric not found', 404, 'NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Update metric
   */
  async updateMetric(metricId: string, metricUpdate: MetricUpdate): Promise<MetricResponse> {
    try {
      const response = await apiService.put<MetricResponse>(`/api/v1/metrics/${metricId}`, metricUpdate);
      
      // Clear cache after update
      this.clearCache();
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to update metric', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to update metric', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Metric not found', 404, 'NOT_FOUND');
      }
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('Metric update validation failed', error.details?.detail);
      }
      throw error;
    }
  }

  /**
   * Delete metric (admin only)
   */
  async deleteMetric(metricId: string): Promise<void> {
    try {
      await apiService.delete(`/api/v1/metrics/${metricId}`);
      
      // Clear cache after deletion
      this.clearCache();
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to delete metric', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Admin access required to delete metrics', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Metric not found', 404, 'NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Get user metrics summary
   */
  async getMetricsSummary(params: { metric_type?: MetricType; start_date?: string; end_date?: string } = {}): Promise<MetricSummary> {
    try {
      const queryParams = new URLSearchParams();
      
      if (params.metric_type) queryParams.append('metric_type', params.metric_type);
      if (params.start_date) queryParams.append('start_date', params.start_date);
      if (params.end_date) queryParams.append('end_date', params.end_date);

      const queryString = queryParams.toString();
      const endpoint = queryString ? `/api/v1/metrics/summary?${queryString}` : '/api/v1/metrics/summary';
      
      const response = await apiService.get<MetricSummary>(endpoint);
      console.log("Metrics summary: ", response);
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access metrics summary', 401, 'UNAUTHORIZED');
      }
      throw error;
    }
  }

  /**
   * Get health prediction
   */
  async getHealthPrediction(params: { include_metrics?: boolean; prediction_horizon_hours?: number } = {}): Promise<HealthPredictionResponse> {
    try {
      const queryParams = new URLSearchParams();
      
      if (params.include_metrics !== undefined) queryParams.append('include_metrics', params.include_metrics.toString());
      if (params.prediction_horizon_hours) queryParams.append('prediction_horizon_hours', params.prediction_horizon_hours.toString());

      const queryString = queryParams.toString();
      const endpoint = queryString ? `/api/v1/metrics/health-prediction?${queryString}` : '/api/v1/metrics/health-prediction';
      
      const response = await apiService.get<HealthPredictionResponse>(endpoint);
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access health prediction', 401, 'UNAUTHORIZED');
      }
      throw error;
    }
  }

  /**
   * Get metrics statistics
   */
  async getMetricsStatistics(params: { start_date?: string; end_date?: string; metric_type?: MetricType } = {}): Promise<MetricsStatistics> {
    try {
      const queryParams = new URLSearchParams();
      
      if (params.start_date) queryParams.append('start_date', params.start_date);
      if (params.end_date) queryParams.append('end_date', params.end_date);
      if (params.metric_type) queryParams.append('metric_type', params.metric_type);

      const queryString = queryParams.toString();
      const endpoint = queryString ? `/api/v1/metrics/statistics?${queryString}` : '/api/v1/metrics/statistics';
      
      const response = await apiService.get<MetricsStatistics>(endpoint);
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access metrics statistics', 401, 'UNAUTHORIZED');
      }
      throw error;
    }
  }

  /**
   * Get metrics by type
   */
  async getMetricsByType(metricType: MetricType, params: Omit<MetricQueryParams, 'metric_type'> = {}): Promise<MetricResponse[]> {
    return this.getMetrics({ ...params, metric_type: metricType });
  }

  /**
   * Get latest metrics
   */
  async getLatestMetrics(limit: number = 10): Promise<MetricResponse[]> {
    return this.getMetrics({ limit, skip: 0 });
  }

  /**
   * Generate cache key for query parameters
   */
  private generateCacheKey(params: MetricQueryParams): string {
    return JSON.stringify(params);
  }

  /**
   * Clear metrics cache
   */
  clearCache(): void {
    this.metricsCache.clear();
  }

  /**
   * Cache metrics for a specific query
   */
  private cacheMetrics(key: string, metrics: MetricResponse[]): void {
    this.metricsCache.set(key, metrics);
  }

  /**
   * Create health metrics from device data
   * Filters out invalid data (0 values from unavailable sensors) before creating metrics
   */
  createHealthMetrics(healthData: {
    heartRate?: number;
    oxygenLevel?: number;
    temperature?: number;
    steps?: number;
    sleep?: number;
    timestamp: string;
    sensorModel: string;
  }): MetricCreate[] {
    const metrics: MetricCreate[] = [];

    // Filter out 0 values (sensor unavailable) and validate ranges for non-zero values
    if (healthData.heartRate !== undefined && healthData.heartRate !== 0) {
      const validation = this.validateMetric({
        metric_type: MetricType.HEART_RATE,
        value: healthData.heartRate,
        unit: 'bpm',
        sensor_model: healthData.sensorModel,
        timestamp: healthData.timestamp,
      });
      
      if (validation.valid) {
        metrics.push({
          metric_type: MetricType.HEART_RATE,
          value: healthData.heartRate,
          unit: 'bpm',
          sensor_model: healthData.sensorModel,
          timestamp: healthData.timestamp,
        });
      } else {
        console.warn('Filtered invalid heart rate:', healthData.heartRate, validation.error);
      }
    }

    if (healthData.oxygenLevel !== undefined && healthData.oxygenLevel !== 0) {
      const validation = this.validateMetric({
        metric_type: MetricType.SPO2,
        value: healthData.oxygenLevel,
        unit: '%',
        sensor_model: healthData.sensorModel,
        timestamp: healthData.timestamp,
      });
      
      if (validation.valid) {
        metrics.push({
          metric_type: MetricType.SPO2,
          value: healthData.oxygenLevel,
          unit: '%',
          sensor_model: healthData.sensorModel,
          timestamp: healthData.timestamp,
        });
      } else {
        console.warn('Filtered invalid SpO2:', healthData.oxygenLevel, validation.error);
      }
    }

    if (healthData.temperature !== undefined && healthData.temperature !== 0) {
      const validation = this.validateMetric({
        metric_type: MetricType.SKIN_TEMPERATURE,
        value: healthData.temperature,
        unit: '°C',
        sensor_model: healthData.sensorModel,
        timestamp: healthData.timestamp,
      });
      
      if (validation.valid) {
        metrics.push({
          metric_type: MetricType.SKIN_TEMPERATURE,
          value: healthData.temperature,
          unit: '°C',
          sensor_model: healthData.sensorModel,
          timestamp: healthData.timestamp,
        });
      } else {
        console.warn('Filtered invalid temperature:', healthData.temperature, validation.error);
      }
    }

    if (healthData.steps !== undefined && healthData.steps !== 0) {
      metrics.push({
        metric_type: MetricType.STEPS,
        value: healthData.steps,
        unit: 'steps',
        sensor_model: healthData.sensorModel,
        timestamp: healthData.timestamp,
      });
    }

    if (healthData.sleep !== undefined && healthData.sleep !== 0) {
      metrics.push({
        metric_type: MetricType.SLEEP,
        value: healthData.sleep,
        unit: 'hours',
        sensor_model: healthData.sensorModel,
        timestamp: healthData.timestamp,
      });
    }

    return metrics;
  }

  /**
   * Validate metric data
   * Allows 0 values (sensor unavailable) while still validating legitimate ranges
   */
  validateMetric(metric: MetricCreate): { valid: boolean; error?: string } {
    // Check required fields
    if (!metric.metric_type) {
      return { valid: false, error: 'metric_type is required' };
    }
    if (!metric.unit || metric.unit.trim() === '') {
      return { valid: false, error: 'unit is required' };
    }
    if (!metric.sensor_model || metric.sensor_model.trim() === '') {
      return { valid: false, error: 'sensor_model is required' };
    }

    // Allow undefined values (optional metrics)
    if (metric.value === undefined) {
      return { valid: true };
    }

    // Allow 0 values (sensor unavailable) but validate non-zero values
    if (metric.value === 0) {
      return { valid: true };
    }

    // Validate value ranges based on metric type for non-zero values
    switch (metric.metric_type) {
      case MetricType.HEART_RATE:
        if (metric.value < 30 || metric.value > 220) {
          return { valid: false, error: `Heart rate value ${metric.value} is outside valid range (30-220 bpm)` };
        }
        break;
      case MetricType.SPO2:
        if (metric.value < 70 || metric.value > 100) {
          return { valid: false, error: `SpO2 value ${metric.value} is outside valid range (70-100%)` };
        }
        break;
      case MetricType.SKIN_TEMPERATURE:
        if (metric.value < 20 || metric.value > 45) {
          return { valid: false, error: `Skin temperature value ${metric.value} is outside valid range (20-45°C)` };
        }
        break;
      case MetricType.AMBIENT_TEMPERATURE:
        if (metric.value < -50 || metric.value > 60) {
          return { valid: false, error: `Ambient temperature value ${metric.value} is outside valid range (-50-60°C)` };
        }
        break;
      case MetricType.STEPS:
        if (metric.value < 0) {
          return { valid: false, error: `Steps value ${metric.value} cannot be negative` };
        }
        break;
      case MetricType.SLEEP:
        if (metric.value < 0 || metric.value > 24) {
          return { valid: false, error: `Sleep value ${metric.value} is outside valid range (0-24 hours)` };
        }
        break;
      default:
        // Unknown metric type - allow any value
        break;
    }

    return { valid: true };
  }

  /**
   * Validate metrics batch
   * Provides detailed error messages for validation failures
   */
  validateMetricsBatch(batch: MetricBatchCreate): { valid: boolean; errors: string[] } {
    const errors: string[] = [];
    
    if (!batch.metrics || !Array.isArray(batch.metrics)) {
      errors.push('Metrics must be an array');
      return { valid: false, errors };
    }

    if (batch.metrics.length === 0) {
      errors.push('Metrics array cannot be empty');
      return { valid: false, errors };
    }

    if (batch.metrics.length > 1000) {
      errors.push('Metrics batch cannot exceed 1000 metrics');
      return { valid: false, errors };
    }

    for (let i = 0; i < batch.metrics.length; i++) {
      const metric = batch.metrics[i];
      const validation = this.validateMetric(metric);
      
      if (!validation.valid) {
        const metricInfo = `[${metric.metric_type}] value: ${metric.value}`;
        errors.push(`Metric at index ${i} (${metricInfo}) is invalid: ${validation.error}`);
      }
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Transform device data structure to OpenAPI-compliant metrics
   * Handles undefined values and field name mapping
   */
  transformDeviceDataToMetrics(deviceData: {
    heartRate?: number;
    sleepHours?: number;
    spo2?: number;
    steps?: number;
    temperature?: number;
    timestamp?: string;
  }, sensorModel: string = 'smart_bracelet_v2'): MetricCreate[] {
    const metrics: MetricCreate[] = [];
    const currentTimestamp = new Date().toISOString();

    // Field mapping configuration
    const fieldMappings = [
      {
        deviceField: 'heartRate' as const,
        metricType: MetricType.HEART_RATE,
        unit: 'bpm',
        validation: (value: number) => value >= 30 && value <= 220
      },
      {
        deviceField: 'sleepHours' as const,
        metricType: MetricType.SLEEP,
        unit: 'hours',
        validation: (value: number) => value >= 0 && value <= 24
      },
      {
        deviceField: 'spo2' as const,
        metricType: MetricType.SPO2,
        unit: '%',
        validation: (value: number) => value >= 70 && value <= 100
      },
      {
        deviceField: 'steps' as const,
        metricType: MetricType.STEPS,
        unit: 'steps',
        validation: (value: number) => value >= 0
      },
      {
        deviceField: 'temperature' as const,
        metricType: MetricType.SKIN_TEMPERATURE,
        unit: '°C',
        validation: (value: number) => value >= 20 && value <= 45
      }
    ];

    // Process each field mapping
    fieldMappings.forEach(mapping => {
      const deviceValue = deviceData[mapping.deviceField];
      
      // Skip undefined values
      if (deviceValue === undefined) {
        return;
      }

      // Validate value if validation function exists
      if (mapping.validation && !mapping.validation(deviceValue)) {
        console.warn(`Filtered invalid ${mapping.deviceField}:`, deviceValue);
        return;
      }

      // Create metric
      const metric: MetricCreate = {
        metric_type: mapping.metricType,
        value: deviceValue,
        unit: mapping.unit,
        sensor_model: sensorModel,
        timestamp: deviceData.timestamp || currentTimestamp
      };

      metrics.push(metric);
    });

    return metrics;
  }

  /**
   * Create metrics batch from device data
   * Complete flow from device data to batch API submission
   */
  async createMetricsBatchFromDeviceData(
    deviceData: {
      heartRate?: number;
      sleepHours?: number;
      spo2?: number;
      steps?: number;
      temperature?: number;
      timestamp?: string;
    },
    sensorModel: string = 'smart_bracelet_v2'
  ): Promise<any> {
    // Transform device data to metrics
    const metrics = this.transformDeviceDataToMetrics(deviceData, sensorModel);
    
    // If no valid metrics, return early
    if (metrics.length === 0) {
      return { status: 'no_valid_metrics', count: 0 };
    }

    // Create batch and submit
    const batch: MetricBatchCreate = { metrics };
    return this.createMetricsBatch(batch);
  }
}

// Singleton instance
export const metricService = new MetricService();