/* eslint-disable no-bitwise */
import { useState } from "react";
import { Alert, Linking, PermissionsAndroid, Platform } from "react-native";

import * as ExpoDevice from "expo-device";
import base64 from 'react-native-base64';
import {
    BleError,
    BleManager,
    Characteristic,
    Device,
    State,
} from "react-native-ble-plx";

// Static UUIDs for Health-Monitor device
const HEALTH_MONITOR_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const HEALTH_MONITOR_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const HEALTH_MONITOR_CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// Standard BLE MTU sizes
const MAX_MTU_SIZE = 512; // Maximum BLE 4.0+ MTU
const DEFAULT_MTU_SIZE = 512; // Minimum BLE MTU

const bleManager = new BleManager();

function useBLE() {
    const [allDevices, setAllDevices] = useState<Device[]>([]);
    const [connectedDevice, setConnectedDevice] = useState<Device | null>(null);
    const [sensorData, setSensorData] = useState({
        heartRate: 0,
        spo2: 0,
        temperature: 0,
        sleepHours: 0,
        steps: 0,
        timestamp: 0
    });
    const [isScanning, setIsScanning] = useState(false);
    const [mtuSize, setMtuSize] = useState<number>(DEFAULT_MTU_SIZE); // Default BLE MTU
    const [discoveredServices, setDiscoveredServices] = useState<string[]>([]);
    const [discoveredCharacteristics, setDiscoveredCharacteristics] = useState<string[]>([]);
    const [bluetoothState, setBluetoothState] = useState<State>(State.Unknown);
    const [hasLocationPermission, setHasLocationPermission] = useState<boolean>(false);
    const [locationServicesEnabled, setLocationServicesEnabled] = useState<boolean>(false);

    const requestAndroid31Permissions = async () => {
        const bluetoothScanPermission = await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
            {
                title: "Location Permission",
                message: "Bluetooth Low Energy requires Location",
                buttonPositive: "OK",
            }
        );
        const bluetoothConnectPermission = await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
            {
                title: "Location Permission",
                message: "Bluetooth Low Energy requires Location",
                buttonPositive: "OK",
            }
        );
        const fineLocationPermission = await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
            {
                title: "Location Permission",
                message: "Bluetooth Low Energy requires Location",
                buttonPositive: "OK",
            }
        );

        return (
            bluetoothScanPermission === "granted" &&
            bluetoothConnectPermission === "granted" &&
            fineLocationPermission === "granted"
        );
    };

    const checkLocationServices = async (): Promise<boolean> => {
        console.log("useBLE: Checking location services...");
        if (Platform.OS === "android") {
            try {
                // For Android, we can check if location services are enabled
                // using a simpler approach without requiring geolocation API
                // We'll assume location services are enabled by default on Android
                // since most modern Android devices have them enabled for BLE scanning
                console.log("useBLE: Android platform - assuming location services enabled for BLE scanning");
                setLocationServicesEnabled(true);
                return true;
            } catch (error) {
                console.log("useBLE: Error checking location services:", error);
                // If we can't check, assume location services are enabled to avoid blocking users
                setLocationServicesEnabled(true);
                return true;
            }
        } else {
            // For iOS, location services are handled differently
            console.log("useBLE: iOS platform - assuming location services enabled");
            setLocationServicesEnabled(true);
            return true;
        }
    };

    const checkLocationPermission = async (): Promise<boolean> => {
        console.log("useBLE: Checking location permission...");
        if (Platform.OS === "android") {
            try {
                console.log("useBLE: Android platform - checking fine location permission");
                const granted = await PermissionsAndroid.check(
                    PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION
                );
                console.log("useBLE: Android location permission result:", granted);
                setHasLocationPermission(granted);
                return granted;
            } catch (error) {
                console.log("useBLE: Error checking location permission:", error);
                setHasLocationPermission(false);
                Alert.alert("Bluetooth Required", "Please turn on Bluetooth to continue.");
                return false;
            }
        } else {
            // For iOS, location permission is handled differently
            // We'll assume it's granted for now as iOS handles this differently
            const stt = await bleManager.state();
            if (stt !== "PoweredOn") {
                Alert.alert(
                    "Bluetooth is off",
                    "Please turn on Bluetooth in Settings",
                    [
                        { text: 'Open Settings', onPress: () => Linking.openSettings() },
                        { text: 'Cancel', style: 'cancel' }
                    ]
                );
            }
            console.log("useBLE: iOS platform - assuming location permission granted");
            setHasLocationPermission(true);
            return true;
        }
    };

    const requestPermissions = async () => {
        if (Platform.OS === "android") {
            if ((ExpoDevice.platformApiLevel ?? -1) < 31) {
                const granted = await PermissionsAndroid.request(
                    PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
                    {
                        title: "Location Permission",
                        message: "Bluetooth Low Energy requires Location to scan for nearby devices",
                        buttonPositive: "OK",
                    }
                );
                const hasPermission = granted === PermissionsAndroid.RESULTS.GRANTED;
                setHasLocationPermission(hasPermission);
                return hasPermission;
            } else {
                const isAndroid31PermissionsGranted =
                    await requestAndroid31Permissions();
                setHasLocationPermission(isAndroid31PermissionsGranted);
                return isAndroid31PermissionsGranted;
            }
        } else {
            // For iOS, permissions are handled differently
            // We'll assume they're granted for now
            setHasLocationPermission(true);
            return true;
        }
    };

    const checkAllPermissions = async (): Promise<{
        bluetoothEnabled: boolean;
        locationPermission: boolean;
        locationServicesEnabled: boolean;
    }> => {
        console.log("useBLE: Starting checkAllPermissions...");
        const bluetoothEnabled = await checkBluetoothState();
        console.log("useBLE: Bluetooth state check result:", bluetoothEnabled);
        const locationPermission = await checkLocationPermission();
        console.log("useBLE: Location permission check result:", locationPermission);
        const locationServicesEnabled = await checkLocationServices();
        console.log("useBLE: Location services check result:", locationServicesEnabled);

        const result = {
            bluetoothEnabled,
            locationPermission,
            locationServicesEnabled
        };
        console.log("useBLE: checkAllPermissions returning:", result);
        return result;
    };

    const connectToDevice = async (device: Device) => {
        try {
            console.log(`Connecting to device ${device.name} ${device.id}`);

            // Stop scanning immediately when connecting to a device
            if (isScanning) {
                bleManager.stopDeviceScan();
                setIsScanning(false);
            }

            // Add connection timeout
            const connectionTimeout = new Promise<never>((_, reject) => {
                setTimeout(() => {
                    const timeoutError = new Error("Connection timeout after 10 seconds");
                    (timeoutError as any).code = "CONNECTION_TIMEOUT";
                    (timeoutError as any).reason = "Connection timeout";
                    reject(timeoutError);
                }, 10000);
            });

            const deviceConnection = await Promise.race([
                bleManager.connectToDevice(device.id),
                connectionTimeout
            ]);

            setConnectedDevice(deviceConnection);

            // Discover all services and characteristics (still needed for device communication)
            await deviceConnection.discoverAllServicesAndCharacteristics();

            // Get discovered services and characteristics for logging
            const services = await deviceConnection.services();
            const serviceIds = services.map(service => service.uuid);
            setDiscoveredServices(serviceIds);

            console.log(`Discovered services: ${serviceIds.join(', ')}`);

            // Discover characteristics for each service for logging
            const allCharacteristics: string[] = [];
            for (const service of services) {
                const characteristics = await deviceConnection.characteristicsForService(service.uuid);
                const charIds = characteristics.map(char => char.uuid);
                allCharacteristics.push(...charIds);
                console.log(`Service ${service.uuid} characteristics: ${charIds.join(', ')}`);
            }
            setDiscoveredCharacteristics(allCharacteristics);

            // Use static UUIDs directly (no dynamic discovery)
            console.log('Using static Health-Monitor BLE UUIDs:', {
                service: HEALTH_MONITOR_SERVICE_UUID,
                tx: HEALTH_MONITOR_CHAR_TX_UUID,
                rx: HEALTH_MONITOR_CHAR_RX_UUID
            });

            // Request optimal MTU with fallback
            try {
                console.log(`Requesting MTU: ${MAX_MTU_SIZE} bytes`);
                await deviceConnection.requestMTU(MAX_MTU_SIZE);
                setMtuSize(MAX_MTU_SIZE);
                console.log(`MTU negotiated: ${MAX_MTU_SIZE} bytes`);
            } catch (mtuError) {
                console.log(`MTU request failed (${MAX_MTU_SIZE} bytes), reason:`, (mtuError as any)?.reason || mtuError);
                try {
                    // Fallback to default MTU
                    console.log(`Falling back to default MTU: ${DEFAULT_MTU_SIZE} bytes`);
                    await deviceConnection.requestMTU(DEFAULT_MTU_SIZE);
                    setMtuSize(DEFAULT_MTU_SIZE);
                    console.log(`MTU negotiated: ${DEFAULT_MTU_SIZE} bytes`);
                } catch (fallbackError) {
                    console.log(`Default MTU request also failed, reason:`, (fallbackError as any)?.reason || fallbackError);
                    console.log("Continuing with platform default MTU");
                }
            }

            startStreamingData(deviceConnection);
        } catch (e) {
            const error = e as any;
            console.log("FAILED TO CONNECT", e);
            console.log("Connection error reason:", error?.reason || error?.message || "Unknown reason");

            // Attempt recovery by disconnecting
            if (device) {
                try {
                    await bleManager.cancelDeviceConnection(device.id);
                } catch (disconnectError) {
                    const disconnectErrorObj = disconnectError as any;
                    console.log("Failed to clean up connection:", disconnectError);
                    console.log("Disconnection error reason:", disconnectErrorObj?.reason || disconnectErrorObj?.message || "Unknown reason");
                }
            }
        }
    };

    const isDuplicteDevice = (devices: Device[], nextDevice: Device) =>
        devices.findIndex((device) => nextDevice.id === device.id) > -1;

    const checkBluetoothState = async (): Promise<boolean> => {
        try {
            console.log("useBLE: Checking Bluetooth state...");

            // Add timeout to prevent hanging
            const timeoutPromise = new Promise<never>((_, reject) => {
                setTimeout(() => {
                    const timeoutError = new Error("Bluetooth state check timeout");
                    (timeoutError as any).code = "BLUETOOTH_STATE_TIMEOUT";
                    (timeoutError as any).reason = "Bluetooth state check timeout";
                    reject(timeoutError);
                }, 3000);
            });

            const state = await Promise.race([
                bleManager.state(),
                timeoutPromise
            ]);

            console.log("useBLE: Bluetooth state received:", state);
            setBluetoothState(state);
            const isPoweredOn = state === State.PoweredOn;
            console.log("useBLE: Bluetooth is powered on:", isPoweredOn);
            return isPoweredOn;
        } catch (error) {
            console.log("useBLE: Error checking Bluetooth state:", error);
            setBluetoothState(State.Unknown);
            return false;
        }
    };

    const scanForPeripherals = async (): Promise<boolean> => {
        if (isScanning) {
            console.log("Scan already in progress");
            return false;
        }

        // Check all permissions before scanning
        const permissions = await checkAllPermissions();

        if (!permissions.bluetoothEnabled) {
            console.log("Bluetooth is not enabled, cannot scan");
            return false;
        }

        if (!permissions.locationPermission) {
            console.log("Location permission not granted, cannot scan");
            return false;
        }

        if (!permissions.locationServicesEnabled) {
            console.log("Location services are disabled, cannot scan");
            Alert.alert(
                "Location Services Required",
                "Bluetooth scanning requires location services to be enabled on Android devices. Please enable location services to scan for devices.",
                [
                    { text: 'Open Location Settings', onPress: () => Linking.openSettings() },
                    { text: 'Cancel', style: 'cancel' }
                ]
            );
            return false;
        }

        setIsScanning(true);
        setAllDevices([]); // Clear previous devices

        // Start scan with timeout
        bleManager.startDeviceScan(null, null, (error, device) => {
            if (error) {
                console.log("BLE Scan Error:", error);
                console.log("BLE Scan error reason:", error.reason || "Unknown reason");
                setIsScanning(false);
                return;
            }

            if (device && device.name) {
                console.log(`Found device: ${device.name} ${device.id}`);

                setAllDevices((prevState: Device[]) => {
                    if (!isDuplicteDevice(prevState, device)) {
                        return [...prevState, device];
                    }
                    return prevState;
                });
            } else if (device) {
                console.log(`Found unnamed device (filtered out): ${device.id}`);
            }
        });

        // Stop scan after 30 seconds
        setTimeout(() => {
            console.log("BLE scan timeout reached (30 seconds) - stopping scan");
            stopScan();
        }, 30000);

        return true;
    };

    const stopScan = () => {
        // if (isScanning) {
        // }
        bleManager.stopDeviceScan();
        setIsScanning(false);
        console.log("BLE scan stopped");
    };

    const disconnectFromDevice = async () => {
        if (connectedDevice) {
            try {
                await bleManager.cancelDeviceConnection(connectedDevice.id);
                console.log("Successfully disconnected from device");
            } catch (error) {
                console.log("Error disconnecting from device:", error);
                console.log("Disconnection error reason:", (error as any)?.reason || "Unknown reason");
            } finally {
                setConnectedDevice(null);
                setSensorData({
                    heartRate: 0,
                    spo2: 0,
                    temperature: 0,
                    sleepHours: 0,
                    steps: 0,
                    timestamp: 0
                });
                setDiscoveredServices([]);
                setDiscoveredCharacteristics([]);
            }
        }
    };

    const onDataUpdate = (
        error: BleError | null,
        characteristic: Characteristic | null
    ) => {
        if (error) {
            console.log("Error receiving data from device", error);
            console.log("Data update error reason:", error.reason || "Unknown reason");

            // Attempt recovery by restarting streaming if device is still connected
            if (connectedDevice) {
                console.log("Attempting to restart data streaming...");
                setTimeout(() => {
                    if (connectedDevice) {
                        startStreamingData(connectedDevice);
                    }
                }, 2000);
            }
            return;
        }
        if (characteristic?.value) {
            //console.log("Raw data received:", characteristic.value);
            const rawData = base64.decode(characteristic.value);
            //console.log("Raw data decoded:", rawData);

            try {
                // Parse JSON data from ESP32 with new format
                const parsedData = JSON.parse(rawData);

                // Log sensor data with timestamps
                const timestamp = new Date().toISOString();
                // console.log(`[${timestamp}] Sensor Data:`, {
                //     heartRate: parsedData.heartRate,
                //     spo2: parsedData.spo2,
                //     temperature: parsedData.temperature,
                //     sleepHours: parsedData.sleepHours,
                //     steps: parsedData.steps,
                //     timestamp: parsedData.timestamp
                // });

                // Update state with parsed data (new format)
                setSensorData({
                    heartRate: parsedData.heartRate || 0,
                    spo2: parsedData.spo2 || 0,
                    temperature: parsedData.temperature || 0,
                    sleepHours: parsedData.sleepHours || 0,
                    steps: parsedData.steps || 0,
                    timestamp: parsedData.timestamp || Date.now()
                });

            } catch (parseError) {
                // If JSON parsing fails, try to parse as CSV format (legacy support)
                console.log("JSON parsing failed, trying CSV format:", parseError);

                const values = rawData.split(',').map(val => parseFloat(val.trim()));
                if (values.length >= 9) {
                    const timestamp = new Date().toISOString();
                    console.log(`[${timestamp}] Sensor Data (CSV):`, {
                        heartRate: values[0],
                        spo2: values[1],
                        temperature: values[2],
                        sleepHours: values[4],
                        steps: values[7],
                        timestamp: values[8] || Date.now()
                    });

                    setSensorData({
                        heartRate: values[0] || 0,
                        spo2: values[1] || 0,
                        temperature: values[2] || 0,
                        sleepHours: values[4] || 0,
                        steps: values[7] || 0,
                        timestamp: values[8] || Date.now()
                    });
                } else {
                    console.log("Invalid data format received:", rawData);
                }
            }
        }
    };

    const startStreamingData = async (device: Device) => {
        if (!device) {
            console.log('No device connected');
            return;
        }

        console.log(`Starting data streaming with static Health-Monitor UUIDs:`, {
            service: HEALTH_MONITOR_SERVICE_UUID,
            tx: HEALTH_MONITOR_CHAR_TX_UUID,
            rx: HEALTH_MONITOR_CHAR_RX_UUID
        });

        try {
            // Add timeout for monitoring operation
            const monitorTimeout = new Promise<never>((_, reject) => {
                setTimeout(() => {
                    const timeoutError = new Error("Monitor characteristic timeout after 5 seconds");
                    (timeoutError as any).code = "MONITOR_TIMEOUT";
                    (timeoutError as any).reason = "Monitor characteristic timeout";
                    reject(timeoutError);
                }, 5000);
            });

            await Promise.race([
                new Promise<void>((resolve, reject) => {
                    device.monitorCharacteristicForService(
                        HEALTH_MONITOR_SERVICE_UUID,
                        HEALTH_MONITOR_CHAR_TX_UUID,
                        (error, characteristic) => {
                            if (error) {
                                console.log("Monitor characteristic error:", error);
                                console.log("Monitor error reason:", error.reason || "Unknown reason");
                                // Create a proper Error object with guaranteed non-null properties
                                const errorMessage = error?.message || error?.reason || "BLE operation failed";
                                const errorCode = error?.errorCode || "BLE_ERROR";
                                const fixedError = new Error(errorMessage);
                                (fixedError as any).code = errorCode;
                                (fixedError as any).reason = error?.reason || "Unknown reason";
                                reject(fixedError);
                            } else {
                                onDataUpdate(error, characteristic);
                                resolve();
                            }
                        }
                    );
                }),
                monitorTimeout
            ]);

            const dummy = base64.encode('1');   // any single byte is enough

            // Add timeout for write operation
            const writeTimeout = new Promise<never>((_, reject) => {
                setTimeout(() => {
                    const timeoutError = new Error("Write characteristic timeout after 5 seconds");
                    (timeoutError as any).code = "WRITE_TIMEOUT";
                    (timeoutError as any).reason = "Write characteristic timeout";
                    reject(timeoutError);
                }, 5000);
            });

            await Promise.race([
                device.writeCharacteristicWithResponseForService(
                    HEALTH_MONITOR_SERVICE_UUID,
                    HEALTH_MONITOR_CHAR_RX_UUID,
                    dummy
                ),
                writeTimeout
            ]);

            console.log("Data streaming started successfully");
        } catch (error) {
            const errorObj = error as any;
            console.log("Failed to start data streaming:", error);
            console.log("Streaming error reason:", errorObj?.reason || errorObj?.message || "Unknown reason");

            // Attempt recovery after delay
            setTimeout(() => {
                if (connectedDevice) {
                    console.log("Retrying data streaming...");
                    startStreamingData(connectedDevice);
                }
            }, 3000);
        }
    };

    return {
        connectToDevice,
        allDevices,
        connectedDevice,
        sensorData,
        mtuSize,
        isScanning,
        bluetoothState,
        hasLocationPermission,
        locationServicesEnabled,
        requestPermissions,
        scanForPeripherals,
        stopScan,
        disconnectFromDevice,
        startStreamingData,
        checkBluetoothState,
        checkAllPermissions,
        checkLocationPermission,
        checkLocationServices,
        discoveredServices,
        discoveredCharacteristics,
    };
}

export default useBLE;