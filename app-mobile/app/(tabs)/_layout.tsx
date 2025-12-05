import { useAuth } from "@/contexts/AuthContext";
import { Tabs, useRouter } from "expo-router";
import { BarChart3, Home, Settings, User } from "lucide-react-native";
import React, { useEffect } from "react";

import colors from "@/constants/colors";
import useBLE from "@/hooks/use-ble";

export default function TabLayout() {
  const router = useRouter();
  const { isAuthenticated, isLoading, user } = useAuth();
  const {
    allDevices,
    connectedDevice,
    connectToDevice,
    sensorData,
    isScanning,
    requestPermissions,
    scanForPeripherals,
    stopScan,
    disconnectFromDevice,
  } = useBLE();
  
  // Redirect to email verification if authenticated but email not verified
  useEffect(() => {
    if (!isLoading && isAuthenticated && user && !user.emailVerified) {
      router.push({
        pathname: '/verify-email',
        params: { email: user.email }
      } as any);
    }
  }, [isAuthenticated, isLoading, user, router]);

  if (isLoading || (isAuthenticated && user && !user.emailVerified)) {
    return null; // Will redirect in useEffect
  }

  return (
    <Tabs
      screenOptions={{
        tabBarActiveTintColor: colors.primary,
        tabBarInactiveTintColor: colors.textMuted,
        headerShown: false,
        tabBarStyle: {
          backgroundColor: colors.white,
          borderTopColor: colors.border,
          borderTopWidth: 1,
        },
        tabBarLabelStyle: {
          fontSize: 12,
          fontWeight: '600' as const,
        },
      }}
    >
      <Tabs.Screen
        name="index"
        options={{
          title: "Home",
          tabBarIcon: ({ color, size }) => <Home size={size} color={color} />,
        }}
      />
      <Tabs.Screen
        name="analytics"
        options={{
          title: "Analytics",
          tabBarIcon: ({ color, size }) => <BarChart3 size={size} color={color} />,
        }}
      />
      <Tabs.Screen
        name="profile"
        options={{
          title: "Profile",
          tabBarIcon: ({ color, size }) => <User size={size} color={color} />,
        }}
      />
      <Tabs.Screen
        name="settings"
        options={{
          title: "Settings",
          tabBarIcon: ({ color, size }) => <Settings size={size} color={color} />,
        }}
      />
    </Tabs>
  );
}
