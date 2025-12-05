/**
 * Core Service Tests
 * Tests the core service functionality without React Native dependencies
 */

import { 
  metricService,
  MetricCreate,
  MetricType,
  MetricBatchCreate
} from '../services';

describe('Metric Service Core Functionality', () => {
  test('should validate metrics correctly', () => {
    const validMetric: MetricCreate = {
      metric_type: MetricType.HEART_RATE,
      value: 75,
      unit: 'bpm',
      sensor_model: 'Test-Sensor-1.0',
      timestamp: new Date().toISOString()
    };

    const validation = metricService.validateMetric(validMetric);
    expect(validation.valid).toBe(true);
  });

  test('should reject invalid heart rate metrics', () => {
    const invalidMetric: MetricCreate = {
      metric_type: MetricType.HEART_RATE,
      value: 300, // Invalid heart rate
      unit: 'bpm',
      sensor_model: 'Test-Sensor-1.0'
    };

    const validation = metricService.validateMetric(invalidMetric);
    expect(validation.valid).toBe(false);
    expect(validation.error).toContain('outside valid range');
  });

  test('should reject invalid SpO2 metrics', () => {
    const invalidMetric: MetricCreate = {
      metric_type: MetricType.SPO2,
      value: 50, // Invalid SpO2
      unit: '%',
      sensor_model: 'Test-Sensor-1.0'
    };

    const validation = metricService.validateMetric(invalidMetric);
    expect(validation.valid).toBe(false);
    expect(validation.error).toContain('outside valid range');
  });

  test('should reject metrics without required fields', () => {
    const invalidMetric: MetricCreate = {
      metric_type: MetricType.HEART_RATE,
      value: 75,
      unit: '', // Empty unit
      sensor_model: 'Test-Sensor-1.0'
    };

    const validation = metricService.validateMetric(invalidMetric);
    expect(validation.valid).toBe(false);
    expect(validation.error).toContain('unit is required');
  });

  test('should create health metrics from device data', () => {
    const healthData = {
      heartRate: 75,
      oxygenLevel: 98,
      temperature: 36.5,
      steps: 5000,
      sleep: 7.5,
      timestamp: new Date().toISOString(),
      sensorModel: 'Health-Monitor-Bracelet'
    };

    const metrics = metricService.createHealthMetrics(healthData);

    expect(metrics).toHaveLength(5); // heart_rate, spo2, skin_temperature, steps, sleep
    expect(metrics[0].metric_type).toBe(MetricType.HEART_RATE);
    expect(metrics[1].metric_type).toBe(MetricType.SPO2);
    expect(metrics[2].metric_type).toBe(MetricType.SKIN_TEMPERATURE);
    expect(metrics[3].metric_type).toBe(MetricType.STEPS);
    expect(metrics[4].metric_type).toBe(MetricType.SLEEP);

    // Verify required fields are present
    metrics.forEach(metric => {
      expect(metric.unit).toBeDefined();
      expect(metric.sensor_model).toBeDefined();
      expect(metric.metric_type).toBeDefined();
    });
  });

  test('should filter out zero values from device data', () => {
    const healthData = {
      heartRate: 0, // Sensor unavailable
      oxygenLevel: 98,
      temperature: 0, // Sensor unavailable
      steps: 5000,
      sleep: 7.5,
      timestamp: new Date().toISOString(),
      sensorModel: 'Health-Monitor-Bracelet'
    };

    const metrics = metricService.createHealthMetrics(healthData);

    // Should only create metrics for non-zero values
    expect(metrics).toHaveLength(3); // spo2, steps, sleep
    expect(metrics[0].metric_type).toBe(MetricType.SPO2);
    expect(metrics[1].metric_type).toBe(MetricType.STEPS);
    expect(metrics[2].metric_type).toBe(MetricType.SLEEP);
  });

  test('should validate batch metrics correctly', () => {
    const batch: MetricBatchCreate = {
      metrics: [{
        metric_type: MetricType.HEART_RATE,
        value: 75,
        unit: 'bpm',
        sensor_model: 'Test-Sensor-1.0',
        timestamp: new Date().toISOString()
      }]
    };

    const validation = metricService.validateMetricsBatch(batch);
    expect(validation.valid).toBe(true);
    expect(validation.errors).toHaveLength(0);
  });

  test('should reject empty batch', () => {
    const batch: MetricBatchCreate = {
      metrics: []
    };

    const validation = metricService.validateMetricsBatch(batch);
    expect(validation.valid).toBe(false);
    expect(validation.errors).toContain('Metrics array cannot be empty');
  });

  test('should reject batch with too many metrics', () => {
    const metrics = Array(1001).fill({
      metric_type: MetricType.HEART_RATE,
      value: 75,
      unit: 'bpm',
      sensor_model: 'Test-Sensor-1.0'
    });

    const batch: MetricBatchCreate = { metrics };

    const validation = metricService.validateMetricsBatch(batch);
    expect(validation.valid).toBe(false);
    expect(validation.errors).toContain('Metrics batch cannot exceed 1000 metrics');
  });
});

describe('OpenAPI Specification Compatibility', () => {
  test('should support all required metric types', () => {
    const requiredTypes = ['spo2', 'heart_rate', 'skin_temperature', 'ambient_temperature', 'steps', 'sleep'];
    const serviceTypes = Object.values(MetricType);

    requiredTypes.forEach(type => {
      expect(serviceTypes).toContain(type);
    });
  });

  test('should have required fields in metric creation', () => {
    const metric: MetricCreate = {
      metric_type: MetricType.HEART_RATE,
      unit: 'bpm',
      sensor_model: 'Test-Sensor'
    };

    expect(metric.metric_type).toBeDefined();
    expect(metric.unit).toBeDefined();
    expect(metric.sensor_model).toBeDefined();
  });

  test('should match OpenAPI metric type enum', () => {
    // Verify that all metric types match the OpenAPI specification
    const openApiTypes = ['spo2', 'heart_rate', 'skin_temperature', 'ambient_temperature', 'steps', 'sleep'];
    const serviceTypes = Object.values(MetricType);

    expect(serviceTypes).toEqual(expect.arrayContaining(openApiTypes));
    expect(openApiTypes).toEqual(expect.arrayContaining(serviceTypes));
  });
});

describe('Device Data Transformation and Batch Transmission', () => {
  test('should transform device data with undefined values correctly', () => {
    const deviceData = {
      heartRate: undefined,
      sleepHours: undefined,
      spo2: undefined,
      steps: undefined,
      temperature: undefined,
      timestamp: undefined
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    // Should filter out all undefined values
    expect(metrics).toHaveLength(0);
  });

  test('should transform device data with some defined values correctly', () => {
    const deviceData = {
      heartRate: 75,
      sleepHours: undefined,
      spo2: 98,
      steps: 5000,
      temperature: undefined,
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    // Should only include defined values
    expect(metrics).toHaveLength(3);
    
    // Verify field name mapping
    expect(metrics[0].metric_type).toBe(MetricType.HEART_RATE);
    expect(metrics[0].value).toBe(75);
    expect(metrics[0].unit).toBe('bpm');
    
    expect(metrics[1].metric_type).toBe(MetricType.SPO2);
    expect(metrics[1].value).toBe(98);
    expect(metrics[1].unit).toBe('%');
    
    expect(metrics[2].metric_type).toBe(MetricType.STEPS);
    expect(metrics[2].value).toBe(5000);
    expect(metrics[2].unit).toBe('steps');
    
    // Verify timestamp is preserved
    metrics.forEach(metric => {
      expect(metric.timestamp).toBe('2023-10-01T12:00:00Z');
    });
  });

  test('should transform device data with all defined values correctly', () => {
    const deviceData = {
      heartRate: 75,
      sleepHours: 7.5,
      spo2: 98,
      steps: 5000,
      temperature: 36.5,
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    // Should include all defined values
    expect(metrics).toHaveLength(5);
    
    // Verify all metric types are present
    const metricTypes = metrics.map(m => m.metric_type);
    expect(metricTypes).toContain(MetricType.HEART_RATE);
    expect(metricTypes).toContain(MetricType.SLEEP);
    expect(metricTypes).toContain(MetricType.SPO2);
    expect(metricTypes).toContain(MetricType.STEPS);
    expect(metricTypes).toContain(MetricType.SKIN_TEMPERATURE);
    
    // Verify field name mapping
    const heartRateMetric = metrics.find(m => m.metric_type === MetricType.HEART_RATE);
    expect(heartRateMetric?.value).toBe(75);
    expect(heartRateMetric?.unit).toBe('bpm');
    
    const sleepMetric = metrics.find(m => m.metric_type === MetricType.SLEEP);
    expect(sleepMetric?.value).toBe(7.5);
    expect(sleepMetric?.unit).toBe('hours');
    
    const spo2Metric = metrics.find(m => m.metric_type === MetricType.SPO2);
    expect(spo2Metric?.value).toBe(98);
    expect(spo2Metric?.unit).toBe('%');
    
    const stepsMetric = metrics.find(m => m.metric_type === MetricType.STEPS);
    expect(stepsMetric?.value).toBe(5000);
    expect(stepsMetric?.unit).toBe('steps');
    
    const tempMetric = metrics.find(m => m.metric_type === MetricType.SKIN_TEMPERATURE);
    expect(tempMetric?.value).toBe(36.5);
    expect(tempMetric?.unit).toBe('°C');
  });

  test('should add current timestamp when not provided', () => {
    const deviceData = {
      heartRate: 75,
      steps: 5000
      // No timestamp provided
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    expect(metrics).toHaveLength(2);
    
    // Should have current timestamp
    metrics.forEach(metric => {
      expect(metric.timestamp).toBeDefined();
      expect(new Date(metric.timestamp!)).toBeInstanceOf(Date);
    });
  });

  test('should use custom sensor model when provided', () => {
    const deviceData = {
      heartRate: 75,
      steps: 5000
    };

    const customSensorModel = 'custom_bracelet_v3';
    const metrics = metricService.transformDeviceDataToMetrics(deviceData, customSensorModel);
    
    expect(metrics).toHaveLength(2);
    
    // Should use custom sensor model
    metrics.forEach(metric => {
      expect(metric.sensor_model).toBe(customSensorModel);
    });
  });

  test('should filter invalid metric values', () => {
    const deviceData = {
      heartRate: 300, // Invalid heart rate
      spo2: 50, // Invalid SpO2
      temperature: 50, // Invalid temperature
      steps: -100, // Invalid steps
      sleepHours: 25, // Invalid sleep hours
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    // Should filter out all invalid values
    expect(metrics).toHaveLength(0);
  });

  test('should handle mixed valid and invalid values', () => {
    const deviceData = {
      heartRate: 300, // Invalid
      spo2: 98, // Valid
      steps: 5000, // Valid
      temperature: 50, // Invalid
      sleepHours: 7.5, // Valid
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    // Should only include valid values
    expect(metrics).toHaveLength(3);
    
    const metricTypes = metrics.map(m => m.metric_type);
    expect(metricTypes).toContain(MetricType.SPO2);
    expect(metricTypes).toContain(MetricType.STEPS);
    expect(metricTypes).toContain(MetricType.SLEEP);
    
    // Should not include invalid values
    expect(metricTypes).not.toContain(MetricType.HEART_RATE);
    expect(metricTypes).not.toContain(MetricType.SKIN_TEMPERATURE);
  });

  test('should create complete batch transmission flow', async () => {
    const deviceData = {
      heartRate: 75,
      sleepHours: 7.5,
      spo2: 98,
      steps: 5000,
      temperature: 36.5,
      timestamp: '2023-10-01T12:00:00Z'
    };

    // Mock the batch creation method
    const mockBatchResult = { status: 'success', count: 5 };
    const mockCreateMetricsBatch = jest.spyOn(metricService, 'createMetricsBatch')
      .mockResolvedValue(mockBatchResult);

    const result = await metricService.createMetricsBatchFromDeviceData(deviceData);
    
    expect(result).toEqual(mockBatchResult);
    expect(mockCreateMetricsBatch).toHaveBeenCalledWith({
      metrics: expect.arrayContaining([
        expect.objectContaining({
          metric_type: MetricType.HEART_RATE,
          value: 75,
          unit: 'bpm',
          sensor_model: 'smart_bracelet_v2',
          timestamp: '2023-10-01T12:00:00Z'
        }),
        expect.objectContaining({
          metric_type: MetricType.SLEEP,
          value: 7.5,
          unit: 'hours',
          sensor_model: 'smart_bracelet_v2',
          timestamp: '2023-10-01T12:00:00Z'
        })
      ])
    });

    mockCreateMetricsBatch.mockRestore();
  });

  test('should handle batch transmission with no valid metrics', async () => {
    const deviceData = {
      heartRate: undefined,
      sleepHours: undefined,
      spo2: undefined,
      steps: undefined,
      temperature: undefined,
      timestamp: undefined
    };

    const result = await metricService.createMetricsBatchFromDeviceData(deviceData);
    
    expect(result).toEqual({ status: 'no_valid_metrics', count: 0 });
  });

  test('should ensure OpenAPI compliance in transformed metrics', () => {
    const deviceData = {
      heartRate: 75,
      sleepHours: 7.5,
      spo2: 98,
      steps: 5000,
      temperature: 36.5,
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);
    
    metrics.forEach(metric => {
      // Verify required OpenAPI fields
      expect(metric.metric_type).toBeDefined();
      expect(metric.unit).toBeDefined();
      expect(metric.sensor_model).toBeDefined();
      
      // Verify metric types match OpenAPI specification
      const validMetricTypes = Object.values(MetricType);
      expect(validMetricTypes).toContain(metric.metric_type);
      
      // Verify units are appropriate for each metric type
      switch (metric.metric_type) {
        case MetricType.HEART_RATE:
          expect(metric.unit).toBe('bpm');
          break;
        case MetricType.SLEEP:
          expect(metric.unit).toBe('hours');
          break;
        case MetricType.SPO2:
          expect(metric.unit).toBe('%');
          break;
        case MetricType.STEPS:
          expect(metric.unit).toBe('steps');
          break;
        case MetricType.SKIN_TEMPERATURE:
          expect(metric.unit).toBe('°C');
          break;
      }
    });
  });
});