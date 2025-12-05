import { issueService, metricService, offlineStorageService, MetricCreate, MetricType } from '@/services';
import createContextHook from '@nkzw/create-context-hook';
import { useCallback, useEffect, useMemo, useState } from 'react';
import { useBle } from './BleContext';
import { useAuth } from './AuthContext';

export interface HealthData {
  heartRate: number;
  sleepHours: number;
  temperature: number;
  oxygenLevel: number;
  steps: number;
  timestamp: number;
  lastUpdated: Date;
}

export interface HistoricalData {
  timestamp: Date;
  heartRate: number;
  oxygenLevel: number;
  temperature: number;
  steps: number;
  sleepHours: number;
}


export interface UserProfile {
  name: string;
  age: number;
  gender: string;
  photoUrl: string;
  bloodType: string;
  conditions: string[];
  allergies: string[];
  medications: string[];
}

export const [HealthDataProvider, useHealthData] = createContextHook(() => {
  const [hasAlerts, setHasAlerts] = useState<boolean>(false);
  const [currentData, setCurrentData] = useState<HealthData>({
    heartRate: 0,
    sleepHours: 0,
    temperature: 0,
    oxygenLevel: 0,
    steps: 0,
    timestamp: 0,
    lastUpdated: new Date(),
  });

  const [historicalData, setHistoricalData] = useState<HistoricalData[]>([]);
  const [healthPrediction, setHealthPrediction] = useState<any | null>(null);
  const [isLoadingPrediction, setIsLoadingPrediction] = useState<boolean>(false);
  const [userProfile, setUserProfile] = useState<UserProfile>({
    name: 'Sarah Johnson',
    age: 32,
    gender: 'Male',
    photoUrl: 'https://i.pravatar.cc/150?img=47',
    bloodType: 'A+',
    conditions: ['Hypertension', 'Asthma'],
    allergies: ['Penicillin', 'Peanuts'],
    medications: ['Lisinopril 10mg', 'Albuterol inhaler'],
  });

  // Use the Bluetooth context
  const {
    connectedDevice,
    isConnected,
    sensorData: bleSensorData,
    connectToDevice,
    disconnectFromDevice,
  } = useBle();

  // Use the Auth context to check authentication state
  const { isAuthenticated } = useAuth();

  // Update health data when Bluetooth data changes
  useEffect(() => {
    if (bleSensorData && isConnected) {
      const newData: HealthData = {
        heartRate: bleSensorData.heartRate || 0,
        sleepHours: bleSensorData.sleepHours || 0, // Now dynamic from device data
        temperature: bleSensorData.temperature || 0,
        oxygenLevel: bleSensorData.spo2 || 0,
        steps: bleSensorData.steps || 0,
        timestamp: bleSensorData.timestamp || Date.now(),
        lastUpdated: new Date(),
      };

      setCurrentData(newData);

      // Add to historical data
      setHistoricalData(prev => {
        const newHistoricalData = [...prev];
        newHistoricalData.push({
          timestamp: new Date(),
          heartRate: bleSensorData.heartRate || 0,
          oxygenLevel: bleSensorData.spo2 || 0,
          temperature: bleSensorData.temperature || 0,
          steps: bleSensorData.steps || 0,
          sleepHours: bleSensorData.sleepHours || 0,
        });
        
        // Keep only last 24 hours of data
        const twentyFourHoursAgo = new Date(Date.now() - 24 * 60 * 60 * 1000);
        return newHistoricalData.filter(item => item.timestamp > twentyFourHoursAgo);
      });

      // Send metrics to backend if device is registered
      sendMetricsToBackend(newData, new Date());

      // Check for alerts based on health data
      // checkForAlerts(newData);
    }
  }, [bleSensorData, isConnected]);

  // Send health metrics to backend or offline storage with authentication awareness
  const sendMetricsToBackend = useCallback(async (data: HealthData, timestamp: Date) => {
    try {
      const metrics = metricService.createHealthMetrics({
        heartRate: data.heartRate,
        oxygenLevel: data.oxygenLevel,
        temperature: data.temperature,
        steps: data.steps,
        sleep: data.sleepHours,
        timestamp: timestamp.toISOString(),
        sensorModel: 'Health-Monitor-Bracelet', // Default sensor model
      });

      if (metrics.length === 0) {
        console.log('No valid metrics to send/store');
        return;
      }

      // Authentication-aware data handling
      if (isAuthenticated) {
        // User is authenticated - send to backend if online, store offline otherwise
        if (offlineStorageService.isOnline()) {
          try {
            await metricService.createMetricsBatch({ metrics });
            console.log('Metrics sent to backend successfully (authenticated user)');
          } catch (apiError) {
            console.error('Failed to send metrics to backend, storing offline:', apiError);
            await offlineStorageService.storeMetrics(metrics);
            console.log('Metrics stored offline after backend failure');
          }
        } else {
          await offlineStorageService.storeMetrics(metrics);
          console.log('Metrics stored offline for later sync (authenticated user)');
        }
      } else {
        // User is not authenticated - always store offline with unauthenticated metadata
        await offlineStorageService.storeMetrics(metrics);
        console.log('Metrics stored offline (unauthenticated user)');
        
        // Clean up old unauthenticated data periodically (7-day retention)
        const storedMetrics = await offlineStorageService.getStoredMetrics();
        if (storedMetrics.length > 100) { // Cleanup when we have significant data
          await offlineStorageService.cleanupOldData(false); // false = unauthenticated retention (7 days)
        }
      }
    } catch (error) {
      console.error('Failed to send/store metrics:', error);
      // Final fallback - try to store offline regardless of authentication state
      try {
        const metrics = metricService.createHealthMetrics({
          heartRate: data.heartRate,
          oxygenLevel: data.oxygenLevel,
          temperature: data.temperature,
          steps: data.steps,
          sleep: data.sleepHours,
          timestamp: timestamp.toISOString(),
          sensorModel: 'Health-Monitor-Bracelet', // Default sensor model
        });
        await offlineStorageService.storeMetrics(metrics);
        console.log('Metrics stored offline as final fallback');
      } catch (fallbackError) {
        console.error('Failed to store metrics offline as fallback:', fallbackError);
      }
    }
  }, [isAuthenticated]);

  const checkForAlerts = useCallback((data: HealthData) => {
    const alerts = [];
    
    if (data.heartRate < 60 || data.heartRate > 100) {
      alerts.push('Abnormal heart rate detected');
    }
    
    if (data.sleepHours < 6) {
      alerts.push('Insufficient sleep detected');
    }
    
    if (data.temperature < 36.0 || data.temperature > 37.5) {
      alerts.push('Abnormal body temperature');
    }
    
    if (data.oxygenLevel < 95) {
      alerts.push('Low oxygen saturation');
    }

    if (alerts.length > 0) {
      setHasAlerts(true);
      
      // Create health issues for critical alerts
      const healthIssue = issueService.createHealthIssueFromMetrics({
        heartRate: data.heartRate,
        oxygenLevel: data.oxygenLevel,
        temperature: data.temperature,
        sleepHours: data.sleepHours,
        timestamp: data.lastUpdated.toISOString(),
      });

      if (healthIssue) {
        // In a real app, you would save this to the backend
        console.log('Health issue detected:', healthIssue);
      }

      // Auto-clear alerts after 10 seconds
      setTimeout(() => setHasAlerts(false), 10000);
    }
  }, []);

  const refreshData = useCallback(async () => {
    console.log('Refreshing health data and fetching health prediction...');
    
    try {
      // Update the timestamp
      setCurrentData(prev => ({
        ...prev,
        lastUpdated: new Date(),
      }));

      // Fetch health prediction if authenticated
      if (isAuthenticated) {
        setIsLoadingPrediction(true);
        try {
          const prediction = await metricService.getHealthPrediction();
          setHealthPrediction(prediction);
          console.log('Health prediction fetched successfully:', prediction);
        } catch (predictionError) {
          console.error('Failed to fetch health prediction:', predictionError);
          // Don't throw error here - prediction failure shouldn't block refresh
        } finally {
          setIsLoadingPrediction(false);
        }
      }

      console.log('Health data refreshed successfully');
    } catch (error) {
      console.error('Failed to refresh health data:', error);
    }
  }, [isAuthenticated]);

  const updateUserProfile = useCallback((updates: Partial<UserProfile>) => {
    setUserProfile((prev) => ({ ...prev, ...updates }));
  }, []);

  const toggleConnection = useCallback(async () => {
    if (isConnected && connectedDevice) {
      // Disconnect from current device
      try {
        await disconnectFromDevice();
      } catch (error) {
        console.error('Error disconnecting device:', error);
      }
    } else {
      // Connection is handled through the device selection modal
      console.log('Use device selection modal to connect');
    }
  }, [isConnected, connectedDevice, disconnectFromDevice]);

  return useMemo(
    () => ({
      hasAlerts,
      currentData,
      historicalData,
      healthPrediction,
      isLoadingPrediction,
      userProfile,
      toggleConnection,
      isConnected,
      connectedDevice,
      connectToDevice,
      disconnectFromDevice,
      refreshData,
      updateUserProfile,
    }),
    [
      hasAlerts,
      currentData,
      historicalData,
      healthPrediction,
      isLoadingPrediction,
      userProfile,
      toggleConnection,
      isConnected,
      connectedDevice,
      connectToDevice,
      disconnectFromDevice,
      refreshData,
      updateUserProfile,
    ]
  );
});
