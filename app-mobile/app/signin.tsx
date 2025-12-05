import React, { useState } from 'react';
import {
  View,
  Text,
  StyleSheet,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  TextInput,
  TouchableOpacity,
  Alert,
  ActivityIndicator,
} from 'react-native';
import { Stack, useRouter } from 'expo-router';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { ArrowLeft, Mail, Lock } from 'lucide-react-native';
import { useAuth } from '@/contexts/AuthContext';
import colors from '@/constants/colors';
import { LoadingSpinner } from '@/components/feedback/LoadingSpinner';

export default function SignInScreen() {
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const { signIn, isLoading } = useAuth();

  const [formData, setFormData] = useState({
    email: 'yusf.works@gmail.com',
    password: 'P@ssw0rd_ze',
  });

  const handleSignIn = async () => {
    if (!formData.email || !formData.password) {
      Alert.alert('Error', 'Please fill in all fields');
      return;
    }

    console.log(formData);
    
    const result = await signIn(formData.email, formData.password);
    
    if (result.success) {
      // Navigate to the main app after successful sign in
      router.replace('/(tabs)');
    }
  };

  const navigateToSignUp = () => {
    router.push('/signup');
  };

  return (
    <>
      <Stack.Screen
        options={{
          headerShown: false,
        }}
      />
      <KeyboardAvoidingView
        style={styles.container}
        behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
        keyboardVerticalOffset={Platform.OS === 'ios' ? 64 : 0}
      >
        <ScrollView contentContainerStyle={styles.contentContainer}>
        <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
          <TouchableOpacity style={styles.backButton} onPress={() => router.push("/(public)")}>
            <ArrowLeft size={24} color={colors.text} />
          </TouchableOpacity>
          <Text style={styles.headerTitle}>Sign In</Text>
          <View style={styles.headerPlaceholder} />
        </View>

        <View style={styles.form}>
          <View style={styles.inputContainer}>
            <View style={styles.inputIcon}>
              <Mail size={20} color={colors.textMuted} />
            </View>
            <TextInput
              style={styles.input}
              placeholder="Email Address"
              placeholderTextColor={colors.textMuted}
              value={formData.email}
              onChangeText={(text) => setFormData(prev => ({ ...prev, email: text }))}
              autoCapitalize="none"
              autoComplete="email"
              keyboardType="email-address"
              editable={!isLoading}
            />
          </View>

          <View style={styles.inputContainer}>
            <View style={styles.inputIcon}>
              <Lock size={20} color={colors.textMuted} />
            </View>
            <TextInput
              style={styles.input}
              placeholder="Password"
              placeholderTextColor={colors.textMuted}
              value={formData.password}
              onChangeText={(text) => setFormData(prev => ({ ...prev, password: text }))}
              secureTextEntry
              autoCapitalize="none"
              autoComplete="current-password"
              editable={!isLoading}
            />
          </View>

          <TouchableOpacity
            style={[styles.signInButton, isLoading && styles.signInButtonDisabled]}
            onPress={handleSignIn}
            disabled={isLoading}
          >
            {isLoading ? (
              <View style={styles.buttonContent}>
                <ActivityIndicator size="small" color={colors.white} />
                <Text style={styles.signInButtonText}>Signing In...</Text>
              </View>
            ) : (
              <Text style={styles.signInButtonText}>Sign In</Text>
            )}
          </TouchableOpacity>

          <View style={styles.signUpContainer}>
            <Text style={styles.signUpText}>Don&apos;t have an account? </Text>
            <TouchableOpacity onPress={navigateToSignUp} disabled={isLoading}>
              <Text style={[styles.signUpLink, isLoading && styles.disabledLink]}>Sign Up</Text>
            </TouchableOpacity>
          </View>
          <View style={styles.signUpContainer}>
            <Text style={styles.signUpText}>Account not verified? </Text>
            <TouchableOpacity onPress={()=>router.push('/verify-email')} disabled={isLoading}>
              <Text style={[styles.signUpLink, isLoading && styles.disabledLink]}>Verify account</Text>
            </TouchableOpacity>
          </View>
        </View>
        </ScrollView>
      </KeyboardAvoidingView>

      {isLoading && (
        <LoadingSpinner
          overlay={true}
          text="Signing in..."
          fullScreen={true}
        />
      )}
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
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingBottom: 40,
  },
  backButton: {
    padding: 8,
  },
  headerTitle: {
    fontSize: 24,
    fontWeight: '700' as const,
    color: colors.text,
  },
  headerPlaceholder: {
    width: 40,
  },
  form: {
    paddingHorizontal: 20,
  },
  inputContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: colors.white,
    borderRadius: 12,
    marginBottom: 16,
    borderWidth: 1,
    borderColor: colors.border,
  },
  inputIcon: {
    padding: 16,
    borderRightWidth: 1,
    borderRightColor: colors.border,
  },
  input: {
    flex: 1,
    padding: 16,
    fontSize: 16,
    color: colors.text,
  },
  signInButton: {
    backgroundColor: colors.primary,
    paddingVertical: 16,
    borderRadius: 12,
    alignItems: 'center',
    marginTop: 24,
    marginBottom: 32,
  },
  signInButtonDisabled: {
    opacity: 0.6,
  },
  signInButtonText: {
    color: colors.white,
    fontSize: 16,
    fontWeight: '600' as const,
  },
  buttonContent: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  signUpContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    marginBottom: 16
  },
  signUpText: {
    fontSize: 14,
    color: colors.textMuted,
  },
  signUpLink: {
    fontSize: 14,
    color: colors.primary,
    fontWeight: '600' as const,
  },
  disabledLink: {
    opacity: 0.5,
  },
});