/**
 * BLE UUID Configuration System
 * 
 * This system provides dynamic UUID configuration for BLE services and characteristics.
 * It supports:
 * - Environment variable configuration
 * - Automatic device discovery
 * - Fallback to default UUIDs
 */

// Static UUIDs for Health-Monitor device (as specified in task)
export const HEALTH_MONITOR_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
export const HEALTH_MONITOR_CHAR_TX_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
export const HEALTH_MONITOR_CHAR_RX_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

// Default UUIDs for ESP32 BLE devices (fallback if discovery fails)
export const DEFAULT_DATA_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
export const DEFAULT_CHAR_UUID_TX = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
export const DEFAULT_CHAR_UUID_RX = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

// Environment variable UUIDs (if provided)
export const ENV_SERVICE_UUID = process.env.EXPO_PUBLIC_BLE_SERVICE_UUID;
export const ENV_CHAR_TX_UUID = process.env.EXPO_PUBLIC_BLE_CHAR_TX_UUID;
export const ENV_CHAR_RX_UUID = process.env.EXPO_PUBLIC_BLE_CHAR_RX_UUID;

/**
 * BLE UUID Configuration Interface
 */
export interface BleUuidConfig {
  serviceUuid: string;
  txCharacteristicUuid: string;
  rxCharacteristicUuid: string;
  source: 'static' | 'environment' | 'discovery' | 'default';
}

/**
 * Get the configured UUIDs based on priority:
 * 1. Static Health-Monitor UUIDs (highest priority)
 * 2. Environment variables
 * 3. Default UUIDs (lowest priority)
 *
 * Note: Dynamic discovery is removed for reliability
 */
export function getBleUuidConfig(): BleUuidConfig {
  // Priority 1: Static Health-Monitor UUIDs
  console.log('Using static Health-Monitor BLE UUIDs');
  return {
    serviceUuid: HEALTH_MONITOR_SERVICE_UUID,
    txCharacteristicUuid: HEALTH_MONITOR_CHAR_TX_UUID,
    rxCharacteristicUuid: HEALTH_MONITOR_CHAR_RX_UUID,
    source: 'static'
  };
}

/**
 * Discover UUIDs from device services and characteristics
 * This function is deprecated - use static UUIDs instead for reliability
 */
export function discoverUuidsFromDevice(
  services: string[],
  characteristics: string[]
): { serviceUuid?: string; txCharacteristicUuid?: string; rxCharacteristicUuid?: string } {
  console.log('UUID discovery is deprecated - using static Health-Monitor UUIDs instead');
  console.log(`Services found: ${services.length}, Characteristics found: ${characteristics.length}`);
  
  // Return empty result to indicate static UUIDs should be used
  return {};
}

/**
 * Validate UUID configuration
 */
export function validateUuidConfig(config: BleUuidConfig): boolean {
  const isValid = Boolean(
    config.serviceUuid &&
    config.txCharacteristicUuid &&
    config.rxCharacteristicUuid &&
    config.serviceUuid.length > 0 &&
    config.txCharacteristicUuid.length > 0 &&
    config.rxCharacteristicUuid.length > 0
  );
  
  if (!isValid) {
    console.error('Invalid UUID configuration:', config);
  }
  
  return isValid;
}