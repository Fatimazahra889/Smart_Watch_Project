import React, { FC, useCallback, useEffect, useState } from "react";
import {
  FlatList,
  Linking,
  ListRenderItemInfo,
  Modal,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { Device } from "react-native-ble-plx";
import { SafeAreaView } from "react-native-safe-area-context";

type DeviceModalListItemProps = {
  item: ListRenderItemInfo<Device>;
  connectToPeripheral: (device: Device) => void;
  closeModal: () => void;
};

type DeviceModalProps = {
  devices: Device[];
  visible: boolean;
  connectToPeripheral: (device: Device) => void;
  closeModal: () => void;
  isScanning: boolean;
  stopScan: () => void;
  bluetoothState?: string;
  checkAllPermissions?: () => Promise<{
    bluetoothEnabled: boolean;
    locationPermission: boolean;
    locationServicesEnabled: boolean;
  }>;
  requestPermissions?: () => Promise<boolean>;
};

const DeviceModalListItem: FC<DeviceModalListItemProps> = (props) => {
  const { item, connectToPeripheral, closeModal } = props;

  const connectAndCloseModal = useCallback(() => {
    connectToPeripheral(item.item);
    closeModal();
  }, [closeModal, connectToPeripheral, item.item]);

  return (
    <TouchableOpacity
      onPress={connectAndCloseModal}
      style={modalStyle.ctaButton}
    >
      <Text style={modalStyle.ctaButtonText}>
        {item.item.name ?? item.item.localName}
      </Text>
    </TouchableOpacity>
  );
};

export const DeviceModal: FC<DeviceModalProps> = (props) => {
  const {
    devices,
    visible,
    connectToPeripheral,
    closeModal,
    isScanning,
    stopScan,
    bluetoothState,
    checkAllPermissions,
    requestPermissions
  } = props;

  const [permissions, setPermissions] = useState<{
    bluetoothEnabled: boolean;
    locationPermission: boolean;
    locationServicesEnabled: boolean;
  }>({
    bluetoothEnabled: false,
    locationPermission: false,
    locationServicesEnabled: false
  });
  const [permissionsChecked, setPermissionsChecked] = useState(false);

  const renderDeviceModalListItem = useCallback(
    (item: ListRenderItemInfo<Device>) => {
      return (
        <DeviceModalListItem
          item={item}
          connectToPeripheral={connectToPeripheral}
          closeModal={closeModal}
        />
      );
    },
    [closeModal, connectToPeripheral]
  );

  const handleStopScanAndClose = () => {
    // if (isScanning) {
    // }
    stopScan();
    closeModal();
  };

  // Check permissions when modal becomes visible
  useEffect(() => {
    let isMounted = true;
    
    const checkPermissions = async () => {
      console.log("BLE Modal: Starting permission check, visible:", visible, "checkAllPermissions:", !!checkAllPermissions);
      
      if (visible) {
        if (checkAllPermissions) {
          try {
            console.log("BLE Modal: Calling checkAllPermissions...");
            
            // Add timeout to prevent hanging
            const timeoutPromise = new Promise<never>((_, reject) => {
              setTimeout(() => reject(new Error("Permission check timeout")), 5000);
            });
            
            const permissionStatus = await Promise.race([
              checkAllPermissions(),
              timeoutPromise
            ]);
            
            if (isMounted) {
              console.log("BLE Modal: Permission check completed:", permissionStatus);
              setPermissions(permissionStatus);
              setPermissionsChecked(true);
              console.log("BLE Modal: permissionsChecked set to true");
            }
          } catch (error) {
            console.log("BLE Modal: Error checking permissions:", error);
            // Always set permissionsChecked to true even on error/timeout
            // to prevent infinite loading state
            if (isMounted) {
              setPermissionsChecked(true);
              console.log("BLE Modal: permissionsChecked set to true (error case)");
            }
          }
        } else {
          // If checkAllPermissions is not available, use default permissions
          // and proceed to show device scanning UI
          console.log("BLE Modal: checkAllPermissions not available, using default permissions");
          if (isMounted) {
            setPermissions({
              bluetoothEnabled: true,
              locationPermission: true,
              locationServicesEnabled: true
            });
            setPermissionsChecked(true);
            console.log("BLE Modal: permissionsChecked set to true (default permissions)");
          }
        }
      }
    };

    checkPermissions();

    // Cleanup function to prevent state updates after unmount
    return () => {
      isMounted = false;
    };
  }, [visible, checkAllPermissions]);

  // Reset permissions check when modal closes
  useEffect(() => {
    if (!visible) {
      setPermissionsChecked(false);
    }
  }, [visible]);

  const getModalTitle = () => {
    if (!permissions.bluetoothEnabled) {
      return "Bluetooth Required";
    } else if (!permissions.locationPermission) {
      return "Location Permission Required";
    } else if (!permissions.locationServicesEnabled) {
      return "Location Services Required";
    } else if (bluetoothState === 'PoweredOff') {
      return "Bluetooth is Disabled";
    } else if (isScanning) {
      return "Scanning for devices...";
    } else {
      return "Tap on a device to connect";
    }
  };

  const handleRequestPermissions = async () => {
    if (requestPermissions) {
      const granted = await requestPermissions();
      if (granted && checkAllPermissions) {
        const permissionStatus = await checkAllPermissions();
        setPermissions(permissionStatus);
      }
    }
  };

  const openSettings = () => {
    Linking.openSettings();
  };

  const getModalContent = () => {
    // Show loading while checking permissions
    if (!permissionsChecked) {
      return (
        <View style={modalStyle.errorContainer}>
          <Text style={modalStyle.errorText}>
            Checking permissions...
          </Text>
        </View>
      );
    }

    // Show Bluetooth disabled message
    if (!permissions.bluetoothEnabled) {
      return (
        <View style={modalStyle.errorContainer}>
          <Text style={modalStyle.errorText}>
            Bluetooth is required to scan for and connect to health monitoring devices.
          </Text>
          <Text style={[modalStyle.errorText, modalStyle.instructionText]}>
            Please enable Bluetooth in your device settings to continue.
          </Text>
          <TouchableOpacity
            style={[modalStyle.ctaButton, modalStyle.permissionButton]}
            onPress={openSettings}
          >
            <Text style={modalStyle.ctaButtonText}>Open Settings</Text>
          </TouchableOpacity>
        </View>
      );
    }

    // Show location permission required message
    if (!permissions.locationPermission) {
      return (
        <View style={modalStyle.errorContainer}>
          <Text style={modalStyle.errorText}>
            Location permission is required to scan for nearby Bluetooth devices.
          </Text>
          <Text style={[modalStyle.errorText, modalStyle.instructionText]}>
            This app needs location access to discover Bluetooth devices in your vicinity.
          </Text>
          <TouchableOpacity
            style={[modalStyle.ctaButton, modalStyle.permissionButton]}
            onPress={handleRequestPermissions}
          >
            <Text style={modalStyle.ctaButtonText}>Grant Location Permission</Text>
          </TouchableOpacity>
        </View>
      );
    }

    // Show location services disabled message
    if (!permissions.locationServicesEnabled) {
      return (
        <View style={modalStyle.errorContainer}>
          <Text style={modalStyle.errorText}>
            Location services are required to scan for nearby Bluetooth devices.
          </Text>
          <Text style={[modalStyle.errorText, modalStyle.instructionText]}>
            On Android devices, Bluetooth scanning requires location services to be enabled, even when location permission is granted.
          </Text>
          <TouchableOpacity
            style={[modalStyle.ctaButton, modalStyle.permissionButton]}
            onPress={openSettings}
          >
            <Text style={modalStyle.ctaButtonText}>Enable Location Services</Text>
          </TouchableOpacity>
        </View>
      );
    }

    // Show Bluetooth powered off message
    if (bluetoothState === 'PoweredOff') {
      return (
        <View style={modalStyle.errorContainer}>
          <Text style={modalStyle.errorText}>
            Please enable Bluetooth on your device to scan for and connect to health monitoring devices.
          </Text>
        </View>
      );
    }

    // Show device list when all permissions are granted
    return (
      <FlatList
        style={modalStyle.flatList}
        contentContainerStyle={modalStyle.modalFlatlistContainer}
        data={devices}
        renderItem={renderDeviceModalListItem}
        keyExtractor={(item) => item.id}
        showsVerticalScrollIndicator={true}
        ListEmptyComponent={
          <View style={modalStyle.emptyContainer}>
            <Text style={modalStyle.emptyText}>
              {isScanning ? "Searching for devices..." : "No devices found"}
            </Text>
          </View>
        }
      />
    );
  };

  return (
    <Modal
      style={modalStyle.modalContainer}
      animationType="slide"
      transparent={false}
      visible={visible}
      onShow={() => {
        console.log("BLE Modal shown - Bluetooth state:", bluetoothState);
      }}
    >
      <SafeAreaView style={modalStyle.modalContainer}>
        <Text style={modalStyle.modalTitleText}>
          {getModalTitle()}
        </Text>
        {getModalContent()}
        <View style={modalStyle.buttonContainer}>
          <TouchableOpacity
            onPress={handleStopScanAndClose}
            style={[modalStyle.ctaButton, modalStyle.stopButton]}
          >
            <Text style={modalStyle.ctaButtonText}>
              {(!permissions.bluetoothEnabled || !permissions.locationPermission || !permissions.locationServicesEnabled || bluetoothState === 'PoweredOff') ? 'Close' : (isScanning ? "Stop Scan & Close" : "Close")}
            </Text>
          </TouchableOpacity>
        </View>
      </SafeAreaView>
    </Modal>
  );
};

const modalStyle = StyleSheet.create({
  modalContainer: {
    flex: 1,
    backgroundColor: "#f2f2f2",
  },
  flatList: {
    flex: 1,
  },
  modalFlatlistContainer: {
    flexGrow: 1,
    paddingBottom: 20,
  },
  modalCellOutline: {
    borderWidth: 1,
    borderColor: "black",
    alignItems: "center",
    marginHorizontal: 20,
    paddingVertical: 15,
    borderRadius: 8,
  },
  modalTitleText: {
    marginTop: 40,
    fontSize: 24,
    fontWeight: "bold",
    marginHorizontal: 20,
    textAlign: "center",
    marginBottom: 20,
  },
  buttonContainer: {
    paddingHorizontal: 20,
    paddingBottom: 20,
  },
  ctaButton: {
    backgroundColor: "#60b5ffff",
    justifyContent: "center",
    alignItems: "center",
    height: 50,
    marginHorizontal: 20,
    marginBottom: 5,
    borderRadius: 8,
  },
  ctaButtonText: {
    fontSize: 18,
    fontWeight: "bold",
    color: "white",
  },
  stopButton: {
    backgroundColor: "#666666",
  },
  errorContainer: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    paddingHorizontal: 40,
  },
  errorText: {
    fontSize: 16,
    textAlign: "center",
    color: "#666666",
    lineHeight: 24,
  },
  emptyContainer: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    paddingHorizontal: 40,
  },
  emptyText: {
    fontSize: 16,
    textAlign: "center",
    color: "#666666",
    lineHeight: 24,
  },
  instructionText: {
    marginTop: 16,
    fontSize: 14,
  },
  permissionButton: {
    marginTop: 20,
    backgroundColor: "#4CAF50",
  },
});

export default DeviceModal;