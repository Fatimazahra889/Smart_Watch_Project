import { DeviceModal } from "@/components/bluetooth/ble-modal";
import colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { useBle } from "@/contexts/BleContext";
import { useHealthData } from "@/contexts/HealthDataContext";
import { useRouter } from "expo-router";
import {
  Bell,
  Bike,
  Bluetooth,
  Droplet,
  Footprints,
  Heart,
  HeartCrack,
  MoonStarIcon,
  RefreshCw,
  Thermometer,
  User,
} from "lucide-react-native";
import React, { useEffect, useRef, useState } from "react";
import {
  Alert,
  Animated,
  Dimensions,
  Linking,
  Platform,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

const { width } = Dimensions.get("window");
const CARD_WIDTH = (width - 60) / 2;

export default function HomeScreen() {
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const {
    hasAlerts,
    currentData,
    refreshData,
    isConnected,
    connectToDevice,
    disconnectFromDevice,
    connectedDevice,
    healthPrediction,
    isLoadingPrediction,
  } = useHealthData();

  const {
    allDevices: devices,
    isScanning,
    scanForPeripherals: startScan,
    stopScan,
    requestPermissions,
    bluetoothState,
    checkBluetoothState,
    checkAllPermissions,
  } = useBle();
  const { isAuthenticated } = useAuth();
  const pulseAnim = useRef(new Animated.Value(1)).current;
  const heartBeatAnim = useRef(new Animated.Value(0)).current;
  const [showDeviceModal, setShowDeviceModal] = useState(false);
  const [hasScanError, setHasScanError] = useState(false);
  const [permissionChecked, setPermissionChecked] = useState(false);

  // Start scanning when modal opens - only run once when modal becomes visible
  useEffect(() => {
    if (showDeviceModal && !isScanning && !hasScanError) {
      const initiateScan = async () => {
        try {
          // Check Bluetooth state first
          const isBluetoothEnabled = await checkBluetoothState();
          if (!isBluetoothEnabled) {
            console.log("Bluetooth is disabled, cannot scan");
            setHasScanError(true);
            return;
          }

          // Request permissions first
          const hasPermissions = await requestPermissions();
          if (hasPermissions) {
            console.log("Starting BLE scan...");
            setHasScanError(false);
            const scanStarted = await startScan();
            if (!scanStarted) {
              setHasScanError(true);
            }
          } else {
            console.log("BLE permissions denied");
            setHasScanError(true);
          }
        } catch (error) {
          console.log("Error starting BLE scan:", error);
          setHasScanError(true);
        }
      };
      initiateScan();
    }
  }, [showDeviceModal]); // Only depend on showDeviceModal to prevent infinite loops

  // Check permissions and Bluetooth state on app start
  useEffect(() => {
    const checkPermissionsOnStart = async () => {
      if (permissionChecked) return;

      try {
        console.log("Checking Bluetooth and Location permissions...");

        // Check Bluetooth state first
        const isBluetoothEnabled = await checkBluetoothState();
        if (!isBluetoothEnabled) {
          console.log("Bluetooth is disabled, requesting permissions...");
          await requestPermissions();
        } else {
          console.log("Bluetooth is enabled");
        }

        setPermissionChecked(true);
      } catch (error) {
        console.log("Error checking permissions on start:", error);
        setPermissionChecked(true);
      }
    };

    checkPermissionsOnStart();
  }, [checkBluetoothState, requestPermissions, permissionChecked]);

  // Reset scan error when modal closes
  useEffect(() => {
    if (!showDeviceModal) {
      setHasScanError(false);
    }
  }, [showDeviceModal]);

  useEffect(() => {
    const pulse = Animated.loop(
      Animated.sequence([
        Animated.timing(pulseAnim, {
          toValue: 1.15,
          duration: 400,
          useNativeDriver: true,
        }),
        Animated.timing(pulseAnim, {
          toValue: 1,
          duration: 400,
          useNativeDriver: true,
        }),
      ])
    );

    const heartBeat = Animated.loop(
      Animated.sequence([
        Animated.timing(heartBeatAnim, {
          toValue: 1,
          duration: 100,
          useNativeDriver: true,
        }),
        Animated.timing(heartBeatAnim, {
          toValue: 0,
          duration: 100,
          useNativeDriver: true,
        }),
        Animated.timing(heartBeatAnim, {
          toValue: 1,
          duration: 100,
          useNativeDriver: true,
        }),
        Animated.timing(heartBeatAnim, {
          toValue: 0,
          duration: 600,
          useNativeDriver: true,
        }),
      ])
    );

    pulse.start();
    heartBeat.start();

    return () => {
      pulse.stop();
      heartBeat.stop();
    };
  }, [pulseAnim, heartBeatAnim]);

  const heartScale = heartBeatAnim.interpolate({
    inputRange: [0, 1],
    outputRange: [1, 1.08],
  });

  const openBluetoothSettings = async () => {
    if (Platform.OS === "android") {
      // Opens directly to Bluetooth settings on Android
      try {
        await Linking.sendIntent("android.settings.BLUETOOTH_SETTINGS");
      } catch (error) {
        Alert.alert("Error", "Cannot open Bluetooth settings");
      }
    } else {
      // iOS: Opens App Settings (Safe for App Store)
      // Users must navigate manually to Bluetooth from here if needed.
      Linking.openSettings();

      /* NOTE: You can use Linking.openURL('App-Prefs:Bluetooth') for a direct link, 
      BUT using 'App-Prefs' is a private API and typically leads to 
      App Store Rejection. Use only for internal/enterprise apps.
      */
    }
  };

  return (
    <>
      <ScrollView
        style={styles.container}
        contentContainerStyle={styles.contentContainer}
      >
        <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
          <View style={styles.headerLeft}>
            <View
              style={[
                styles.statusDot,
                isConnected ? styles.connected : styles.disconnected,
              ]}
            />
            <TouchableOpacity
              style={styles.statusContainer}
              onPress={() => {
                if (bluetoothState === "PoweredOff") {
                  Alert.alert(
                    "Bluetooth required",
                    "Please enable Bluetooth to connect to your device.",
                    [
                      {
                        text: "Cancel",
                        style: "cancel",
                        onPress: () => {
                          return;
                        },
                      },
                      {
                        text: "Open Settings",
                        onPress: async () => {
                          // Open device Bluetooth settings
                          // Note: This requires additional implementation depending on the platform
                          await openBluetoothSettings();
                        },
                      },
                    ]
                  );
                  return;
                }
                if (isConnected) {
                  Alert.alert(
                    "Disconnect Device",
                    "Do you want to disconnect from the current device?",
                    [
                      { text: "Cancel", style: "cancel" },
                      {
                        text: "Disconnect",
                        style: "destructive",
                        onPress: disconnectFromDevice,
                      },
                    ]
                  );
                } else {
                  setShowDeviceModal(true);
                }
              }}
            >
              {bluetoothState === "PoweredOff" ? (
                <View style={styles.permissionWarning}>
                  <Text style={styles.permissionWarningText}>
                    Activate Bluetooth
                  </Text>
                </View>
              ) : (
                <>
                  <Text style={styles.statusText}>
                    {isConnected ? "Connected" : "Tap to Connect"}
                  </Text>
                  {!isConnected && (
                    <Bluetooth
                      size={16}
                      color={colors.primary}
                      style={styles.bluetoothIcon}
                    />
                  )}
                </>
              )}
            </TouchableOpacity>
          </View>
          <View style={styles.headerRight}>
            {hasAlerts && (
              <View style={styles.alertBadge}>
                <Bell size={20} color={colors.white} />
              </View>
            )}
            <TouchableOpacity
              style={styles.iconButton}
              onPress={() => {
                if (isAuthenticated) {
                  // Navigate to Profile screen if authenticated
                  router.push("/(tabs)/profile");
                  // <Redirect href="/(tabs)/profile" />
                } else {
                  // Navigate to Sign Up screen if not authenticated
                  router.push("/signup");
                  // <Redirect href="/signin" />
                }
              }}
            >
              <User size={24} color={colors.text} />
            </TouchableOpacity>
          </View>
        </View>


        {/* Health prediction widget */}
        <View style={styles.anomalySection}>
          <View style={styles.anomalyHeader}>
            <Text style={styles.anomalyTitle}>Health Status</Text>
            <TouchableOpacity
              onPress={refreshData}
              style={styles.refreshButton}
              disabled={isLoadingPrediction}
            >
              <RefreshCw
                size={20}
                color={isLoadingPrediction ? colors.textMuted : colors.primary}
              />
            </TouchableOpacity>
          </View>
          
          {isLoadingPrediction ? (
            <Text style={styles.anomalyTextLoading}>
              Analyzing health data...
            </Text>
          ) : healthPrediction ? (
            <View>
              <View style={styles.healthRiskRow}>
                <Text style={[
                  styles.healthRiskLevel,
                  healthPrediction.health_risk_level === 'low' && styles.healthRiskLow,
                  healthPrediction.health_risk_level === 'medium' && styles.healthRiskMedium,
                  healthPrediction.health_risk_level === 'high' && styles.healthRiskHigh,
                ]}>
                  {healthPrediction.health_risk_level.toUpperCase()} RISK
                </Text>
                <Text style={styles.confidenceScore}>
                  {Math.round(healthPrediction.confidence_score * 100)}% confidence
                </Text>
              </View>
              {healthPrediction.recommendations.length > 0 && (
                <Text style={styles.recommendationText}>
                  {healthPrediction.recommendations[0]}
                </Text>
              )}
            </View>
          ) : (
            <Text
              style={
                isConnected ? styles.anomalyText : styles.anomalyTextUnavailable
              }
            >
              {isConnected
                ? "All vitals within normal range"
                : "Data unavailable: connect device"}
            </Text>
          )}
        </View>

        <View style={styles.heartSection}>
          {isConnected ? (
            <Animated.View
              style={[
                styles.heartContainer,
                {
                  transform: [
                    { scale: Animated.multiply(pulseAnim, heartScale) },
                  ],
                },
              ]}
            >
              <View style={styles.heartGlow} />
              <Heart size={120} color={colors.heart} fill={colors.heart} />
              <View style={styles.bpmBadge}>
                <View style={styles.bpmContainer}>
                  <Text style={styles.bpmText}>
                    {Math.round(currentData.heartRate)}
                  </Text>
                  <Text style={styles.bpmLabel}>BPM</Text>
                </View>
              </View>
            </Animated.View>
          ) : (
            <View style={styles.heartContainer}>
              <HeartCrack size={120} color={colors.textMuted} />
              <View style={styles.bpmBadge}>
                <View style={styles.bpmContainer}>
                  <Text style={[styles.bpmText, styles.disconnectedBpmText]}>
                    --
                  </Text>
                  <Text style={styles.bpmLabel}>BPM</Text>
                </View>
              </View>
            </View>
          )}
        </View>

        <View style={styles.statsGrid}>
          <View style={styles.statCard}>
            <View
              style={[styles.statIcon, { backgroundColor: colors.secondary }]}
            >
              <MoonStarIcon size={24} color={colors.primary} />
            </View>
            <Text style={styles.statLabel}>Sleep</Text>
            <View style={styles.numUnitWrapper}>
              <Text style={styles.statValue}>
                {currentData.sleepHours.toFixed(1)}
              </Text>
              <Text style={styles.statUnit}>Hours</Text>
            </View>
          </View>

          <View style={styles.statCard}>
            <View style={[styles.statIcon, { backgroundColor: "#FFF4E6" }]}>
              <Thermometer size={24} color={colors.warning} />
            </View>
            <Text style={styles.statLabel}>Body Temperature</Text>
            <View style={styles.numUnitWrapper}>
              <Text style={styles.statValue}>
                {currentData.temperature.toFixed(1)}
              </Text>
              <Text style={styles.statUnit}>°C</Text>
            </View>
          </View>

          <View style={styles.statCard}>
            <View style={[styles.statIcon, { backgroundColor: "#E8F8F5" }]}>
              <Droplet size={24} color={colors.success} />
            </View>
            <Text style={styles.statLabel}>Oxygen</Text>
            <View style={styles.numUnitWrapper}>
              <Text style={styles.statValue}>
                {Math.round(currentData.oxygenLevel)}
              </Text>
              <Text style={styles.statUnit}>%</Text>
            </View>
          </View>

          <View style={styles.statCard}>
            <View style={[styles.statIcon, { backgroundColor: "#ffe6e6ff" }]}>
              <Footprints size={24} color={colors.danger} />
            </View>
            <Text style={styles.statLabel}>Steps</Text>
            <View style={styles.numUnitWrapper}>
              <Text style={styles.statValue}>{currentData.steps}</Text>
              <Text style={styles.statUnit}>steps/H</Text>
            </View>
          </View>
        </View>

        <View style={styles.lastUpdated}>
          <Text style={styles.lastUpdatedText}>
            Last updated: {currentData.lastUpdated.toLocaleTimeString()}
          </Text>
          {connectedDevice && (
            <Text style={styles.deviceInfo}>
              Connected to: {connectedDevice.name}
            </Text>
          )}
        </View>

        {!isConnected && (
          <View style={styles.connectPrompt}>
            <Bluetooth size={48} color={colors.textMuted} />
            <Text style={styles.connectPromptTitle}>No Device Connected</Text>
            <Text style={styles.connectPromptText}>
              Connect your health monitoring device to see real-time data
            </Text>
            <TouchableOpacity
              style={styles.connectButton}
              onPress={() => setShowDeviceModal(true)}
            >
              <Bluetooth size={20} color={colors.white} />
              <Text style={styles.connectButtonText}>Connect Device</Text>
            </TouchableOpacity>
          </View>
        )}
      </ScrollView>

      <DeviceModal
        visible={showDeviceModal}
        closeModal={() => setShowDeviceModal(false)}
        connectToPeripheral={connectToDevice}
        devices={devices}
        isScanning={isScanning}
        stopScan={stopScan}
        bluetoothState={bluetoothState}
        checkAllPermissions={checkAllPermissions}
        requestPermissions={requestPermissions}
      />
    </>
  );
}

const styles = StyleSheet.create({
  numUnitWrapper: {
    flexDirection: "row",
    alignItems: "baseline",
    gap: 4,
  },
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  contentContainer: {
    paddingBottom: 32,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingTop: 60,
    paddingBottom: 20,
  },
  headerLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  statusContainer: {
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
  },
  bluetoothIcon: {
    marginLeft: 4,
  },
  statusDot: {
    width: 10,
    height: 10,
    borderRadius: 5,
  },
  connected: {
    backgroundColor: colors.success,
  },
  disconnected: {
    backgroundColor: colors.danger,
  },
  statusText: {
    fontSize: 14,
    color: colors.text,
    fontWeight: "600" as const,
  },
  headerRight: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  alertBadge: {
    backgroundColor: colors.danger,
    borderRadius: 20,
    padding: 8,
  },
  iconButton: {
    padding: 8,
  },
  anomalySection: {
    marginHorizontal: 20,
    marginBottom: 24,
    padding: 16,
    backgroundColor: colors.white,
    borderRadius: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  anomalyHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 8,
  },
  anomalyTitle: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: colors.text,
  },
  refreshButton: {
    padding: 4,
  },
  anomalyText: {
    fontSize: 14,
    color: colors.success,
    fontWeight: "500" as const,
  },
  anomalyTextUnavailable: {
    fontSize: 14,
    color: colors.secondary,
    fontWeight: "500" as const,
  },
  anomalyTextLoading: {
    fontSize: 14,
    color: colors.primary,
    fontWeight: "500" as const,
  },
  healthRiskRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 8,
  },
  healthRiskLevel: {
    fontSize: 14,
    fontWeight: '700' as const,
    paddingHorizontal: 12,
    paddingVertical: 4,
    borderRadius: 12,
    overflow: 'hidden',
  },
  healthRiskLow: {
    backgroundColor: colors.success + '20',
    color: colors.success,
  },
  healthRiskMedium: {
    backgroundColor: colors.warning + '20',
    color: colors.warning,
  },
  healthRiskHigh: {
    backgroundColor: colors.danger + '20',
    color: colors.danger,
  },
  confidenceScore: {
    fontSize: 12,
    color: colors.textMuted,
    fontWeight: '500' as const,
  },
  recommendationText: {
    fontSize: 12,
    color: colors.text,
    fontWeight: '400' as const,
    lineHeight: 16,
  },
  heartSection: {
    alignItems: "center",
    marginVertical: 32,
  },
  heartContainer: {
    position: "relative",
    alignItems: "center",
    justifyContent: "center",
  },
  heartGlow: {
    position: "absolute",
    width: 160,
    height: 160,
    borderRadius: 80,
    backgroundColor: colors.heartPulse,
    opacity: 0.2,
  },
  bpmBadge: {
    position: "absolute",
    bottom: -10,
    right: -10,
    backgroundColor: colors.white,
    borderRadius: 24,
    paddingHorizontal: 16,
    paddingVertical: 8,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.15,
    shadowRadius: 8,
    elevation: 4,
  },
  bpmContainer: {
    flexDirection: "row",
    alignItems: "baseline",
    gap: 4,
  },
  bpmText: {
    fontSize: 20,
    fontWeight: "700" as const,
    color: colors.text,
  },
  bpmLabel: {
    fontSize: 10,
    color: colors.textMuted,
    fontWeight: "600" as const,
  },
  statsGrid: {
    flexDirection: "row",
    flexWrap: "wrap",
    paddingHorizontal: 20,
    gap: 16,
  },
  statCard: {
    width: CARD_WIDTH,
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  statIcon: {
    width: 48,
    height: 48,
    borderRadius: 24,
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 12,
  },
  statLabel: {
    fontSize: 12,
    color: colors.textMuted,
    marginBottom: 4,
    fontWeight: "500" as const,
  },
  statValue: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: colors.text,
    marginBottom: 2,
  },
  statUnit: {
    fontSize: 12,
    color: colors.textLight,
    fontWeight: "500" as const,
  },
  placeholderCard: {
    alignItems: "center",
    justifyContent: "center",
    borderWidth: 2,
    borderColor: colors.border,
    borderStyle: "dashed" as const,
    backgroundColor: "transparent",
  },
  placeholderText: {
    fontSize: 32,
    color: colors.textMuted,
    fontWeight: "300" as const,
  },
  placeholderLabel: {
    fontSize: 12,
    color: colors.textMuted,
    marginTop: 4,
    fontWeight: "500" as const,
  },
  lastUpdated: {
    alignItems: "center",
    marginTop: 24,
    gap: 4,
  },
  lastUpdatedText: {
    fontSize: 12,
    color: colors.textMuted,
  },
  deviceInfo: {
    fontSize: 11,
    color: colors.textLight,
    fontStyle: "italic",
  },
  connectPrompt: {
    alignItems: "center",
    padding: 32,
    marginHorizontal: 20,
    backgroundColor: colors.white,
    borderRadius: 16,
    marginTop: 24,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  connectPromptTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: colors.text,
    marginTop: 16,
    marginBottom: 8,
  },
  connectPromptText: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: "center",
    marginBottom: 24,
    lineHeight: 20,
  },
  connectButton: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
    backgroundColor: colors.primary,
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 12,
  },
  connectButtonText: {
    color: colors.white,
    fontSize: 16,
    fontWeight: "600",
  },
  disconnectedBpmText: {
    color: colors.textMuted,
  },
  permissionWarning: {
    // position: 'absolute',
    // top: -8,
    // right: -8,
    backgroundColor: colors.warning,
    borderRadius: 10,
    paddingHorizontal: 6,
    paddingVertical: 2,
  },
  permissionWarningText: {
    fontSize: 10,
    color: colors.white,
    fontWeight: "600",
  },
});
