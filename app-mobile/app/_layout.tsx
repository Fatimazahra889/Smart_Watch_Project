import ErrorBoundary from "@/components/error-boundary/ErrorBoundary";
import { AuthProvider, useAuth } from "@/contexts/AuthContext";
import { BleProvider } from "@/contexts/BleContext";
import { HealthDataProvider } from "@/contexts/HealthDataContext";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { Stack, useRouter } from "expo-router";
import * as SplashScreen from "expo-splash-screen";
import React, { useEffect } from "react";
import { GestureHandlerRootView } from "react-native-gesture-handler";
import VerificationPrompt from "@/components/VerificationPrompt";

SplashScreen.preventAutoHideAsync();

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      retry: 2,
      staleTime: 5 * 60 * 1000, // 5 minutes
    },
    mutations: {
      retry: 1,
    },
  },
});

function RootLayoutNav() {
  const { isAuthenticated, isLoading, user } = useAuth();
  const router = useRouter();

  // Redirect to email verification if authenticated but email not verified
  useEffect(() => {
    if (!isLoading && isAuthenticated && user && !user.emailVerified) {
      router.push({
        pathname: '/verify-email',
        params: { email: user.email }
      } as any);
    }
  }, [isAuthenticated, isLoading, user, router]);

  if (isLoading) {
    return null; // Show loading state or splash screen
  }

  return (
    <>
      <Stack screenOptions={{ headerShown: false }}>
        {isAuthenticated && <Stack.Screen name="(tabs)" />}
        {!isAuthenticated && <Stack.Screen name="(public)" />}
      </Stack>

      {/* ✔ Now allowed because it's outside the Stack but inside the layout tree */}
      {isAuthenticated && user && !user.emailVerified && (
        <VerificationPrompt />
      )}
    </>
  );
}

export default function RootLayout() {

  useEffect(() => {
    SplashScreen.hideAsync();
  }, []);

  return (
    <ErrorBoundary>
      <QueryClientProvider client={queryClient}>
        <AuthProvider>
          <BleProvider>
            <HealthDataProvider>
              <GestureHandlerRootView style={{ flex: 1 }}>
                <RootLayoutNav />
              </GestureHandlerRootView>
            </HealthDataProvider>
          </BleProvider>
        </AuthProvider>
      </QueryClientProvider>
    </ErrorBoundary>
  );
}
