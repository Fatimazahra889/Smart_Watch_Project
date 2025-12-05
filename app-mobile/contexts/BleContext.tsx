import useBLE from '@/hooks/use-ble';
import createContextHook from '@nkzw/create-context-hook';
import { useCallback, useEffect, useMemo, useState } from 'react';
import { Device } from 'react-native-ble-plx';

export interface BleState {
  allDevices: Device[];
  connectedDevice: Device | null;
  sensorData: {
    heartRate: number;
    spo2: number;
    temperature: number;
    sleepHours: number;
    steps: number;
    timestamp: number;
  };
  isScanning: boolean;
  isConnected: boolean;
  connectionError: string | null;
  bluetoothState: string;
  hasLocationPermission: boolean;
  locationServicesEnabled: boolean;
}

export interface BleActions {
  connectToDevice: (device: Device) => Promise<void>;
  disconnectFromDevice: () => void;
  requestPermissions: () => Promise<boolean>;
  scanForPeripherals: () => Promise<boolean>;
  stopScan: () => void;
  checkBluetoothState: () => Promise<boolean>;
  checkAllPermissions: () => Promise<{
    bluetoothEnabled: boolean;
    locationPermission: boolean;
    locationServicesEnabled: boolean;
  }>;
  checkLocationPermission: () => Promise<boolean>;
}

export const [BleProvider, useBle] = createContextHook(() => {
  const {
    allDevices,
    connectedDevice,
    sensorData,
    isScanning,
    bluetoothState,
    hasLocationPermission,
    locationServicesEnabled,
    connectToDevice,
    requestPermissions,
    scanForPeripherals,
    stopScan,
    disconnectFromDevice,
    startStreamingData,
    checkBluetoothState,
    checkAllPermissions,
    checkLocationPermission,
  } = useBLE();

  const [connectionError, setConnectionError] = useState<string | null>(null);

  // Clear connection error when connected
  useEffect(() => {
    if (connectedDevice) {
      setConnectionError(null);
    }
  }, [connectedDevice]);

  const handleConnectToDevice = useCallback(async (device: Device) => {
    try {
      setConnectionError(null);
      await connectToDevice(device);
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Connection failed';
      setConnectionError(errorMessage);
      console.error('BLE connection error:', error);
    }
  }, [connectToDevice]);

  const handleDisconnectFromDevice = useCallback(() => {
    setConnectionError(null);
    disconnectFromDevice();
  }, [disconnectFromDevice]);

  const state: BleState = useMemo(() => ({
    allDevices,
    connectedDevice,
    sensorData,
    isScanning,
    isConnected: !!connectedDevice,
    connectionError,
    bluetoothState,
    hasLocationPermission,
    locationServicesEnabled,
  }), [allDevices, connectedDevice, sensorData, isScanning, connectionError, bluetoothState, hasLocationPermission, locationServicesEnabled]);

  const actions: BleActions = useMemo(() => ({
    connectToDevice: handleConnectToDevice,
    disconnectFromDevice: handleDisconnectFromDevice,
    requestPermissions,
    scanForPeripherals,
    stopScan,
    checkBluetoothState,
    checkAllPermissions,
    checkLocationPermission,
  }), [handleConnectToDevice, handleDisconnectFromDevice, requestPermissions, scanForPeripherals, stopScan, checkBluetoothState, checkAllPermissions, checkLocationPermission]);

  return useMemo(() => ({
    ...state,
    ...actions,
  }), [state, actions]);
});