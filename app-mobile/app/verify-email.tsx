import colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { Stack, useLocalSearchParams, useRouter } from "expo-router";
import { ArrowLeft, Mail, RefreshCw, LogOut } from "lucide-react-native";
import React, { useEffect, useState } from "react";
import {
  Alert,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

export default function VerifyEmailScreen() {
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const params = useLocalSearchParams();
  const {
    user,
    verifyEmail,
    resendVerificationCode,
    signOut,
    isLoading,
    isAuthenticated,
  } = useAuth();

  const [code, setCode] = useState("");
  const [isVerifying, setIsVerifying] = useState(false);
  const [isResending, setIsResending] = useState(false);
  const [countdown, setCountdown] = useState(0);

  const email = (params.email as string) || user?.email || "";

  useEffect(() => {
    if (countdown > 0) {
      const timer = setTimeout(() => setCountdown(countdown - 1), 1000);
      return () => clearTimeout(timer);
    }
  }, [countdown]);

  const handleVerify = async () => {
    if (!code.trim()) {
      Alert.alert("Error", "Please enter the verification code");
      return;
    }

    if (code.length !== 6) {
      Alert.alert("Error", "Verification code must be 6 digits");
      return;
    }

    setIsVerifying(true);
    const result = await verifyEmail(code);
    setIsVerifying(false);

    if (result.success) {
      Alert.alert("Success", "Email verified successfully!", [
        { text: "OK", onPress: () => router.replace("/signin") },
      ]);
    } else {
      Alert.alert("Error", result.error || "Failed to verify email");
    }
  };

  const handleResendCode = async () => {
    setIsResending(true);
    const result = await resendVerificationCode();
    setIsResending(false);

    if (result.success) {
      setCountdown(60); // 60 seconds cooldown
      Alert.alert("Success", "Verification code sent to your email");
    } else {
      Alert.alert(
        "Error",
        result.error || "Failed to resend verification code"
      );
    }
  };

  const canResend = countdown === 0 && !isResending;

  return (
    <>
      <Stack.Screen
        options={{
          headerShown: false,
        }}
      />
      <KeyboardAvoidingView
        style={styles.container}
        behavior={Platform.OS === "ios" ? "padding" : "height"}
        keyboardVerticalOffset={Platform.OS === "ios" ? 64 : 0}
      >
        <ScrollView contentContainerStyle={styles.contentContainer}>
          <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
            <TouchableOpacity
              style={styles.backButton}
              onPress={() => router.back()}
            >
              <ArrowLeft size={24} color={colors.text} />
            </TouchableOpacity>
            <Text style={styles.headerTitle}>Verify Email</Text>
            {isAuthenticated && (
              <TouchableOpacity style={styles.logoutButton} onPress={signOut}>
                <LogOut size={20} color={colors.danger} />
              </TouchableOpacity>
            )}
          </View>

          <View style={styles.content}>
            <View style={styles.iconContainer}>
              <Mail size={64} color={colors.primary} />
            </View>

            <Text style={styles.title}>Check Your Email</Text>

            <Text style={styles.description}>
              We&apos;ve sent a 6-digit verification code to:
            </Text>

            <Text style={styles.emailText}>{email}</Text>

            <Text style={styles.instruction}>
              Enter the code below to verify your email address
            </Text>

            <View style={styles.codeInputContainer}>
              <TextInput
                style={styles.codeInput}
                placeholder="000000"
                placeholderTextColor={colors.textMuted}
                value={code}
                onChangeText={setCode}
                keyboardType="number-pad"
                maxLength={6}
                textAlign="center"
                autoFocus
              />
            </View>

            <TouchableOpacity
              style={[
                styles.verifyButton,
                (isVerifying || !code.trim()) && styles.verifyButtonDisabled,
              ]}
              onPress={handleVerify}
              disabled={isVerifying || !code.trim()}
            >
              <Text style={styles.verifyButtonText}>
                {isVerifying ? "Verifying..." : "Verify Email"}
              </Text>
            </TouchableOpacity>

            <View style={styles.resendContainer}>
              <Text style={styles.resendText}>
                Didn&apos;t receive the code?{" "}
              </Text>
              <TouchableOpacity
                onPress={handleResendCode}
                disabled={!canResend}
              >
                <Text
                  style={[
                    styles.resendLink,
                    !canResend && styles.resendLinkDisabled,
                  ]}
                >
                  {canResend ? "Resend Code" : `Resend in ${countdown}s`}
                </Text>
              </TouchableOpacity>
            </View>

            {isResending && (
              <View style={styles.resendLoading}>
                <RefreshCw size={16} color={colors.textMuted} />
                <Text style={styles.resendLoadingText}>Sending code...</Text>
              </View>
            )}
          </View>
        </ScrollView>
      </KeyboardAvoidingView>
    </>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  contentContainer: {
    flexGrow: 1,
    paddingBottom: 32,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingBottom: 40,
  },
  backButton: {
    padding: 8,
  },
  headerTitle: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: colors.text,
  },
  logoutButton: {
    padding: 8,
    backgroundColor: colors.white,
    borderRadius: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  content: {
    paddingHorizontal: 20,
    alignItems: "center",
  },
  iconContainer: {
    width: 120,
    height: 120,
    borderRadius: 60,
    backgroundColor: colors.primary + "20",
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 32,
  },
  title: {
    fontSize: 28,
    fontWeight: "700" as const,
    color: colors.text,
    textAlign: "center",
    marginBottom: 16,
  },
  description: {
    fontSize: 16,
    color: colors.textMuted,
    textAlign: "center",
    marginBottom: 8,
    lineHeight: 22,
  },
  emailText: {
    fontSize: 18,
    fontWeight: "600" as const,
    color: colors.primary,
    textAlign: "center",
    marginBottom: 24,
  },
  instruction: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: "center",
    marginBottom: 32,
    lineHeight: 20,
  },
  codeInputContainer: {
    width: "100%",
    marginBottom: 32,
  },
  codeInput: {
    backgroundColor: colors.white,
    borderWidth: 2,
    borderColor: colors.border,
    borderRadius: 12,
    padding: 20,
    fontSize: 24,
    fontWeight: "600" as const,
    color: colors.text,
    letterSpacing: 8,
  },
  verifyButton: {
    backgroundColor: colors.primary,
    paddingVertical: 16,
    borderRadius: 12,
    alignItems: "center",
    width: "100%",
    marginBottom: 24,
  },
  verifyButtonDisabled: {
    opacity: 0.6,
  },
  verifyButtonText: {
    color: colors.white,
    fontSize: 16,
    fontWeight: "600" as const,
  },
  resendContainer: {
    flexDirection: "row",
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 16,
  },
  resendText: {
    fontSize: 14,
    color: colors.textMuted,
  },
  resendLink: {
    fontSize: 14,
    color: colors.primary,
    fontWeight: "600" as const,
  },
  resendLinkDisabled: {
    color: colors.textMuted,
  },
  resendLoading: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  resendLoadingText: {
    fontSize: 14,
    color: colors.textMuted,
  },
});
