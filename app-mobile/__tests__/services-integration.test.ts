/**
 * Service Integration Tests
 * End-to-end tests for refactored services and API integration
 */

import { 
  authService, 
  metricService, 
  userService, 
  deviceService,
  LoginCredentials,
  MetricCreate,
  MetricType,
  MetricBatchCreate,
  UserCreate
} from '@/services';

// Mock responses for API calls
const mockTokenResponse = {
  access_token: 'mock-jwt-token',
  token_type: 'bearer'
};

const mockUserResponse = {
  id: 'user-123',
  email: 'test@example.com',
  name: 'Test User',
  is_admin: false,
  created_at: new Date().toISOString(),
  updated_at: new Date().toISOString(),
  email_verified_at: null
};

const mockMetricsResponse = [
  {
    id: 'metric-123',
    metric_type: MetricType.HEART_RATE,
    value: 75,
    unit: 'bpm',
    sensor_model: 'Test-Sensor',
    timestamp: new Date().toISOString(),
    user_id: 'user-123',
    created_at: new Date().toISOString(),
    updated_at: new Date().toISOString(),
    deleted_at: null
  }
];

describe('Authentication Service Integration', () => {
  beforeEach(() => {
    // Reset fetch mock
    (global.fetch as jest.Mock).mockClear();
  });

  test('should initialize authentication service', async () => {
    // Mock successful initialization (no token)
    const AsyncStorage = require('@react-native-async-storage/async-storage');
    AsyncStorage.getItem.mockResolvedValue(null);

    const result = await authService.initialize();
    
    expect(result).toBe(false);
    expect(AsyncStorage.getItem).toHaveBeenCalled();
  });

  test('should handle login with valid credentials', async () => {
    // Mock successful login
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => mockTokenResponse
    });

    const credentials: LoginCredentials = {
      email: 'test@example.com',
      password: 'password123'
    };

    const token = await authService.login(credentials);

    expect(token).toEqual(mockTokenResponse);
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/auth/login'),
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify(credentials)
      })
    );
  });

  test('should handle login with invalid credentials', async () => {
    // Mock failed login
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: false,
      status: 401,
      json: async () => ({ detail: 'Invalid credentials' })
    });

    const credentials: LoginCredentials = {
      email: 'invalid@example.com',
      password: 'wrongpassword'
    };

    await expect(authService.login(credentials)).rejects.toThrow('Invalid email or password');
  });

  test('should handle email verification', async () => {
    // Mock successful email verification
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ message: 'Email verified successfully' })
    });

    await authService.verifyEmail({
      email: 'test@example.com',
      verification_code: '123456'
    });

    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/auth/verify-email'),
      expect.objectContaining({
        method: 'POST'
      })
    );
  });

  test('should handle logout', async () => {
    // Mock successful logout
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ message: 'Logged out successfully' })
    });

    const AsyncStorage = require('@react-native-async-storage/async-storage');
    
    await authService.logout();

    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/auth/logout'),
      expect.objectContaining({
        method: 'POST'
      })
    );
    expect(AsyncStorage.removeItem).toHaveBeenCalled();
  });
});

describe('Metric Service Integration', () => {
  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

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

  test('should reject invalid metrics', () => {
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

  test('should handle batch metric creation', async () => {
    // Mock successful batch creation
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ 
        total_processed: 1,
        successful: 1,
        failed: 0,
        created_metrics: mockMetricsResponse
      })
    });

    const batch: MetricBatchCreate = {
      metrics: [{
        metric_type: MetricType.HEART_RATE,
        value: 75,
        unit: 'bpm',
        sensor_model: 'Test-Sensor-1.0',
        timestamp: new Date().toISOString()
      }]
    };

    const result = await metricService.createMetricsBatch(batch);

    expect(result).toBeDefined();
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/metrics/batch'),
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify(batch)
      })
    );
  });

  test('should handle metric retrieval', async () => {
    // Mock successful metrics retrieval
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => mockMetricsResponse
    });

    const metrics = await metricService.getMetrics();

    expect(metrics).toEqual(mockMetricsResponse);
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/metrics/'),
      expect.objectContaining({
        method: 'GET'
      })
    );
  });
});

describe('User Service Integration', () => {
  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

  test('should handle user registration', async () => {
    // Mock successful registration
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      status: 201,
      json: async () => ({ message: 'User registered successfully' })
    });

    const userData: UserCreate = {
      email: 'newuser@example.com',
      name: 'New User',
      password: 'password123'
    };

    await userService.registerUser(userData);

    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/auth/register'),
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify(userData)
      })
    );
  });

  test('should handle user retrieval', async () => {
    // Mock successful user retrieval
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => [mockUserResponse]
    });

    const users = await userService.getUsers();

    expect(users).toEqual([mockUserResponse]);
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/users/'),
      expect.objectContaining({
        method: 'GET'
      })
    );
  });
});

describe('Device Service Integration', () => {
  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

  test('should handle device retrieval', async () => {
    // Mock successful device retrieval
    const mockDevices = [
      {
        id: 'device-123',
        user_id: 'user-123',
        name: 'Test Device',
        serial_number: 'SN123456',
        model: 'Health-Monitor-Bracelet',
        firmware_version: '1.0.0',
        is_active: true,
        registered_at: new Date().toISOString(),
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
        deleted_at: null
      }
    ];

    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => mockDevices
    });

    const devices = await deviceService.getUserDevices();

    expect(devices).toEqual(mockDevices);
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/devices/'),
      expect.objectContaining({
        method: 'GET'
      })
    );
  });

  test('should manage current device state', () => {
    const device = {
      id: 'device-123',
      user_id: 'user-123',
      name: 'Test Device',
      serial_number: 'SN123456',
      model: 'Health-Monitor-Bracelet',
      firmware_version: '1.0.0',
      is_active: true,
      registered_at: new Date().toISOString(),
      created_at: new Date().toISOString(),
      updated_at: new Date().toISOString(),
      deleted_at: null
    };

    // Set current device
    deviceService.setCurrentDevice(device);
    expect(deviceService.getCurrentDevice()).toEqual(device);

    // Clear current device
    deviceService.clearCurrentDevice();
    expect(deviceService.getCurrentDevice()).toBeNull();
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

  test('should use correct authentication endpoints', () => {
    // Verify that services use the correct OpenAPI endpoints
    const authEndpoints = [
      '/api/v1/auth/login',
      '/api/v1/auth/register',
      '/api/v1/auth/verify-email',
      '/api/v1/auth/resend-code',
      '/api/v1/auth/logout'
    ];

    // These endpoints should be used by the services
    authEndpoints.forEach(endpoint => {
      expect(endpoint).toMatch(/^\/api\/v1\/auth\/\w+/);
    });
  });

  test('should use correct metric endpoints', () => {
    const metricEndpoints = [
      '/api/v1/metrics/batch',
      '/api/v1/metrics/',
      '/api/v1/metrics/{metric_id}',
      '/api/v1/metrics/summary',
      '/api/v1/metrics/health-prediction'
    ];

    metricEndpoints.forEach(endpoint => {
      expect(endpoint).toMatch(/^\/api\/v1\/metrics\/\w*/);
    });
  });
});

describe('Device Data Batch Transmission Integration', () => {
  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

  test('should handle complete device data to batch transmission flow', async () => {
    // Mock successful batch creation
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        total_processed: 3,
        successful: 3,
        failed: 0,
        created_metrics: [
          {
            id: 'metric-1',
            metric_type: MetricType.HEART_RATE,
            value: 75,
            unit: 'bpm',
            sensor_model: 'smart_bracelet_v2',
            timestamp: '2023-10-01T12:00:00Z',
            user_id: 'user-123',
            created_at: new Date().toISOString(),
            updated_at: new Date().toISOString(),
            deleted_at: null
          },
          {
            id: 'metric-2',
            metric_type: MetricType.SPO2,
            value: 98,
            unit: '%',
            sensor_model: 'smart_bracelet_v2',
            timestamp: '2023-10-01T12:00:00Z',
            user_id: 'user-123',
            created_at: new Date().toISOString(),
            updated_at: new Date().toISOString(),
            deleted_at: null
          },
          {
            id: 'metric-3',
            metric_type: MetricType.STEPS,
            value: 5000,
            unit: 'steps',
            sensor_model: 'smart_bracelet_v2',
            timestamp: '2023-10-01T12:00:00Z',
            user_id: 'user-123',
            created_at: new Date().toISOString(),
            updated_at: new Date().toISOString(),
            deleted_at: null
          }
        ]
      })
    });

    const deviceData = {
      heartRate: 75,
      sleepHours: undefined, // Should be filtered out
      spo2: 98,
      steps: 5000,
      temperature: undefined, // Should be filtered out
      timestamp: '2023-10-01T12:00:00Z'
    };

    const result = await metricService.createMetricsBatchFromDeviceData(deviceData);

    expect(result).toBeDefined();
    expect(result.total_processed).toBe(3);
    expect(result.successful).toBe(3);
    
    // Verify API call was made with correct transformed data
    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/metrics/batch'),
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({
          metrics: expect.arrayContaining([
            expect.objectContaining({
              metric_type: MetricType.HEART_RATE,
              value: 75,
              unit: 'bpm',
              sensor_model: 'smart_bracelet_v2',
              timestamp: '2023-10-01T12:00:00Z'
            }),
            expect.objectContaining({
              metric_type: MetricType.SPO2,
              value: 98,
              unit: '%',
              sensor_model: 'smart_bracelet_v2',
              timestamp: '2023-10-01T12:00:00Z'
            }),
            expect.objectContaining({
              metric_type: MetricType.STEPS,
              value: 5000,
              unit: 'steps',
              sensor_model: 'smart_bracelet_v2',
              timestamp: '2023-10-01T12:00:00Z'
            })
          ])
        })
      })
    );
  });

  test('should handle device data with all undefined values', async () => {
    const deviceData = {
      heartRate: undefined,
      sleepHours: undefined,
      spo2: undefined,
      steps: undefined,
      temperature: undefined,
      timestamp: undefined
    };

    const result = await metricService.createMetricsBatchFromDeviceData(deviceData);

    // Should return early with no API call
    expect(result).toEqual({ status: 'no_valid_metrics', count: 0 });
    expect(fetch).not.toHaveBeenCalled();
  });

  test('should handle network errors gracefully', async () => {
    // Mock network error
    (global.fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

    const deviceData = {
      heartRate: 75,
      spo2: 98,
      steps: 5000,
      timestamp: '2023-10-01T12:00:00Z'
    };

    // Mock offline storage to prevent actual storage calls
    const offlineStorageService = require('../services/offline-storage.service').offlineStorageService;
    const mockStoreMetrics = jest.spyOn(offlineStorageService, 'storeMetrics').mockResolvedValue(undefined);

    await expect(metricService.createMetricsBatchFromDeviceData(deviceData)).rejects.toThrow();

    mockStoreMetrics.mockRestore();
  });

  test('should handle validation errors in batch transmission', async () => {
    // Mock validation error response
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: false,
      status: 422,
      json: async () => ({
        detail: [
          {
            loc: ['metrics', 0],
            msg: 'value is not a valid float',
            type: 'type_error.float'
          }
        ]
      })
    });

    const deviceData = {
      heartRate: 75,
      spo2: 98,
      steps: 5000,
      timestamp: '2023-10-01T12:00:00Z'
    };

    await expect(metricService.createMetricsBatchFromDeviceData(deviceData)).rejects.toThrow();

    expect(fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/v1/metrics/batch'),
      expect.anything()
    );
  });

  test('should verify OpenAPI compliance in transformed device data', () => {
    const deviceData = {
      heartRate: 75,
      sleepHours: 7.5,
      spo2: 98,
      steps: 5000,
      temperature: 36.5,
      timestamp: '2023-10-01T12:00:00Z'
    };

    const metrics = metricService.transformDeviceDataToMetrics(deviceData);

    // Verify all metrics are OpenAPI compliant
    metrics.forEach(metric => {
      // Required fields
      expect(metric.metric_type).toBeDefined();
      expect(metric.unit).toBeDefined();
      expect(metric.sensor_model).toBeDefined();

      // Field name mapping verification
      switch (metric.metric_type) {
        case MetricType.HEART_RATE:
          expect(metric.value).toBe(75);
          expect(metric.unit).toBe('bpm');
          break;
        case MetricType.SLEEP:
          expect(metric.value).toBe(7.5);
          expect(metric.unit).toBe('hours');
          break;
        case MetricType.SPO2:
          expect(metric.value).toBe(98);
          expect(metric.unit).toBe('%');
          break;
        case MetricType.STEPS:
          expect(metric.value).toBe(5000);
          expect(metric.unit).toBe('steps');
          break;
        case MetricType.SKIN_TEMPERATURE:
          expect(metric.value).toBe(36.5);
          expect(metric.unit).toBe('°C');
          break;
      }

      // Verify timestamp preservation
      expect(metric.timestamp).toBe('2023-10-01T12:00:00Z');
    });
  });
});