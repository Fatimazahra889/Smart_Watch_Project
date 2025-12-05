import { authService, LoginCredentials, EmailVerificationRequest, ResendCodeRequest, userService, UserCreate } from '@/services';
import createContextHook from '@nkzw/create-context-hook';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useCallback, useEffect, useMemo, useState } from 'react';
import { Alert } from 'react-native';

export interface User {
  id: string;
  email: string;
  name: string;
  createdAt: Date;
  emailVerified: boolean;
}

export interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  isInitialized: boolean;
}

const USER_STORAGE_KEY = '@auth_user';

export const [AuthProvider, useAuth] = createContextHook(() => {
  const [authState, setAuthState] = useState<AuthState>({
    user: null,
    isAuthenticated: false,
    isLoading: true,
    isInitialized: false,
  });

  const initializeAuth = useCallback(async () => {
    try {
      // Initialize authentication service
      const isAuthenticated = await authService.initialize();
      console.log("AuthService initialized, isAuthenticated:", isAuthenticated);
      if (isAuthenticated) {
        // Get user info from token
        const userInfo = await authService.getUserInfo();
        
        if (userInfo) {
          // Try to load additional user data from storage
          const storedUser = await loadStoredUser();
          
          const user: User = storedUser || {
            id: userInfo.sub,
            email: userInfo.email,
            name: userInfo.email.split('@')[0], // Fallback name from email
            createdAt: new Date(userInfo.iat * 1000),
            emailVerified: true, // Assume verified if we have a valid token
          };

          setAuthState({
            user,
            isAuthenticated: true,
            isLoading: false,
            isInitialized: true,
          });
        } else {
          // Token exists but can't decode user info
          await authService.logout();
          setAuthState({
            user: null,
            isAuthenticated: false,
            isLoading: false,
            isInitialized: true,
          });
        }
      } else {
        // No valid authentication
        setAuthState({
          user: null,
          isAuthenticated: false,
          isLoading: false,
          isInitialized: true,
        });
      }
    } catch (error) {
      console.error('Error initializing authentication:', error);
      setAuthState({
        user: null,
        isAuthenticated: false,
        isLoading: false,
        isInitialized: true,
      });
    }
  }, []);

  // Load stored user and initialize authentication on app start
  useEffect(() => {
    initializeAuth();
  }, []);

  const loadStoredUser = useCallback(async (): Promise<User | null> => {
    try {
      const storedUser = await AsyncStorage.getItem(USER_STORAGE_KEY);
      if (storedUser) {
        const userData = JSON.parse(storedUser);
        return {
          ...userData,
          createdAt: new Date(userData.createdAt),
        };
      }
      return null;
    } catch (error) {
      console.error('Error loading stored user:', error);
      return null;
    }
  }, []);

  const storeUser = useCallback(async (user: User): Promise<void> => {
    try {
      await AsyncStorage.setItem(USER_STORAGE_KEY, JSON.stringify(user));
    } catch (error) {
      console.error('Error storing user:', error);
    }
  }, []);

  const clearStoredUser = useCallback(async (): Promise<void> => {
    try {
      await AsyncStorage.removeItem(USER_STORAGE_KEY);
    } catch (error) {
      console.error('Error clearing stored user:', error);
    }
  }, []);

  const signUp = useCallback(async (email: string, name: string, password: string) => {
    try {
      // Register new user using user service
      const userData: UserCreate = {
        email,
        name,
        password,
      };

      await userService.registerUser(userData);

      // Create user object for local state
      const user: User = {
        id: '', // Will be set after email verification and login
        email,
        name,
        createdAt: new Date(),
        emailVerified: false, // New users need email verification
      };

      // Store user data
      await storeUser(user);

      setAuthState({
        user,
        isAuthenticated: false, // User is created but not authenticated yet
        isLoading: false,
        isInitialized: true,
      });

      console.log('User signed up successfully:', email);
      return { success: true };
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Sign up failed';
      Alert.alert('Sign Up Error', message);
      return { success: false, error: message };
    }
  }, [storeUser]);

  const signIn = useCallback(async (email: string, password: string) => {
    try {
      // Use real authentication service
      const credentials: LoginCredentials = {
        email: email,
        password: password,
      };

      const token = await authService.login(credentials);
      
      // Get user info from token
      const userInfo = await authService.getUserInfo();
      if (!userInfo) {
        throw new Error('Failed to get user information from token');
      }

      // Try to load existing user data, or create from token
      let user = await loadStoredUser();
      if (!user) {
        user = {
          id: userInfo.sub,
          email,
          name: email.split('@')[0], // Generate name from email
          createdAt: new Date(userInfo.iat * 1000),
          emailVerified: true, // Assume existing users are verified
        };
        await storeUser(user);
      }

      setAuthState({
        user,
        isAuthenticated: true,
        isLoading: false,
        isInitialized: true,
      });

      console.log('User signed in successfully:', email);
      return { success: true };
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Sign in failed';
      Alert.alert('Sign In Error', message);
      return { success: false, error: message };
    }
  }, [loadStoredUser, storeUser]);

  const signOut = useCallback(async () => {
    try {
      // Clear authentication service
      await authService.logout();
      
      // Clear stored user data
      await clearStoredUser();
      
      setAuthState({
        user: null,
        isAuthenticated: false,
        isLoading: false,
        isInitialized: true,
      });

      console.log('User signed out successfully');
    } catch (error) {
      console.error('Error signing out:', error);
      Alert.alert('Error', 'Failed to sign out');
    }
  }, [clearStoredUser]);

  const updateUser = useCallback(async (updates: Partial<User>) => {
    try {
      if (!authState.user) return;

      const updatedUser = { ...authState.user, ...updates };
      await storeUser(updatedUser);

      setAuthState(prev => ({
        ...prev,
        user: updatedUser,
      }));
    } catch (error) {
      console.error('Error updating user:', error);
      Alert.alert('Error', 'Failed to update profile');
    }
  }, [authState.user, storeUser]);

  const verifyEmail = useCallback(async (code: string): Promise<{ success: boolean; error?: string }> => {
    try {
      if (!authState.user) {
        return { success: false, error: 'No user found' };
      }

      // Call the API to verify email using auth service
      const verifyRequest: EmailVerificationRequest = {
        email: authState.user.email,
        verification_code: code,
      };

      await authService.verifyEmail(verifyRequest);
      
      // Update local user state with verified status
      const updatedUser = { ...authState.user, emailVerified: true };
      await storeUser(updatedUser);

      setAuthState(prev => ({
        ...prev,
        user: updatedUser,
      }));

      return { success: true };
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Email verification failed';
      return { success: false, error: message };
    }
  }, [authState.user, storeUser]);

  const resendVerificationCode = useCallback(async (): Promise<{ success: boolean; error?: string }> => {
    try {
      if (!authState.user) {
        return { success: false, error: 'No user found' };
      }

      // Call the API to resend verification code using auth service
      const resendRequest: ResendCodeRequest = {
        email: authState.user.email,
      };

      await authService.resendVerificationCode(resendRequest);
      
      return { success: true };
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to resend verification code';
      return { success: false, error: message };
    }
  }, [authState.user]);

  const refreshUserData = useCallback(async () => {
    try {
      if (!authState.user) return;

      // Validate token using JWT validation
      const isValid = await authService.validateToken();
      if (!isValid) {
        await signOut();
        return;
      }

      // For JWT tokens, we rely on the token payload for user info
      // No need to fetch user data from backend separately
      const userInfo = await authService.getUserInfo();
      if (userInfo && authState.user) {
        const updatedUser = {
          ...authState.user,
          email: userInfo.email || authState.user.email,
          name: userInfo.name || authState.user.name,
        };
        
        await storeUser(updatedUser);
        setAuthState(prev => ({
          ...prev,
          user: updatedUser,
        }));
      }
    } catch (error) {
      console.error('Error refreshing user data:', error);
    }
  }, [authState.user, signOut, storeUser]);

  return useMemo(
    () => ({
      ...authState,
      signUp,
      signIn,
      signOut,
      updateUser,
      verifyEmail,
      resendVerificationCode,
      refreshUserData,
    }),
    [authState, signUp, signIn, signOut, updateUser, verifyEmail, resendVerificationCode, refreshUserData]
  );
});