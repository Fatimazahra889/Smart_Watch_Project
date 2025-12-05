import AsyncStorage from '@react-native-async-storage/async-storage';
import * as SecureStore from 'expo-secure-store';
import { Platform } from 'react-native';

export interface TokenPair {
  accessToken: string;
  refreshToken: string;
  expiresAt: number; // Unix timestamp in seconds
  issuedAt: number; // Unix timestamp in seconds
}

export interface DecodedToken {
  sub: string; // User ID
  email: string;
  exp: number; // Expiration timestamp
  iat: number; // Issued at timestamp
  [key: string]: any; // Additional claims
}

// Storage keys
const ACCESS_TOKEN_KEY = 'access_token';
const REFRESH_TOKEN_KEY = 'refresh_token';
const TOKEN_EXPIRY_KEY = 'token_expiry';
const TOKEN_ISSUED_KEY = 'token_issued';

/**
 * Secure token storage service for JWT tokens
 * Uses Expo SecureStore on mobile, falls back to AsyncStorage on web
 */
export class TokenService {
  private isSecureStorageAvailable: boolean;

  constructor() {
    // Check if SecureStore is available (mobile devices)
    this.isSecureStorageAvailable = Platform.OS !== 'web';
  }

  /**
   * Store token pair securely
   */
  async storeTokens(tokenPair: TokenPair): Promise<void> {
    try {
      if (this.isSecureStorageAvailable) {
        // Use SecureStore for mobile
        await SecureStore.setItemAsync(ACCESS_TOKEN_KEY, tokenPair.accessToken);
        await SecureStore.setItemAsync(REFRESH_TOKEN_KEY, tokenPair.refreshToken);
        await SecureStore.setItemAsync(TOKEN_EXPIRY_KEY, tokenPair.expiresAt.toString());
        await SecureStore.setItemAsync(TOKEN_ISSUED_KEY, tokenPair.issuedAt.toString());
      } else {
        // Fallback to AsyncStorage for web
        await AsyncStorage.setItem(ACCESS_TOKEN_KEY, tokenPair.accessToken);
        await AsyncStorage.setItem(REFRESH_TOKEN_KEY, tokenPair.refreshToken);
        await AsyncStorage.setItem(TOKEN_EXPIRY_KEY, tokenPair.expiresAt.toString());
        await AsyncStorage.setItem(TOKEN_ISSUED_KEY, tokenPair.issuedAt.toString());
      }
    } catch (error) {
      console.error('Error storing tokens:', error);
      throw new Error('Failed to store authentication tokens');
    }
  }

  /**
   * Retrieve access token
   */
  async getAccessToken(): Promise<string | null> {
    try {
      let token: string | null = null;
      
      if (this.isSecureStorageAvailable) {
        console.log("Checking SecureStore for access token");
        token = await SecureStore.getItemAsync(ACCESS_TOKEN_KEY);
      } else {
        console.log("Checking AsyncStorage for access token");
        token = await AsyncStorage.getItem(ACCESS_TOKEN_KEY);
      }
      
      console.log("Token retrieval result:", token ? `Token available (${token.length} chars)` : "No token available");
      return token;
    } catch (error) {
      console.error('Error retrieving access token:', error);
      return null;
    }
  }

  /**
   * Check if access token is available
   */
  async hasAccessToken(): Promise<boolean> {
    const token = await this.getAccessToken();
    return token !== null && token.trim() !== '';
  }

  /**
   * Retrieve refresh token
   */
  async getRefreshToken(): Promise<string | null> {
    try {
      if (this.isSecureStorageAvailable) {
        return await SecureStore.getItemAsync(REFRESH_TOKEN_KEY);
      } else {
        return await AsyncStorage.getItem(REFRESH_TOKEN_KEY);
      }
    } catch (error) {
      console.error('Error retrieving refresh token:', error);
      return null;
    }
  }

  /**
   * Retrieve token expiry timestamp
   */
  async getTokenExpiry(): Promise<number | null> {
    try {
      let expiry: string | null;
      if (this.isSecureStorageAvailable) {
        expiry = await SecureStore.getItemAsync(TOKEN_EXPIRY_KEY);
      } else {
        expiry = await AsyncStorage.getItem(TOKEN_EXPIRY_KEY);
      }
      
      return expiry ? parseInt(expiry, 10) : null;
    } catch (error) {
      console.error('Error retrieving token expiry:', error);
      return null;
    }
  }

  /**
   * Retrieve token issued timestamp
   */
  async getTokenIssued(): Promise<number | null> {
    try {
      let issued: string | null;
      if (this.isSecureStorageAvailable) {
        issued = await SecureStore.getItemAsync(TOKEN_ISSUED_KEY);
      } else {
        issued = await AsyncStorage.getItem(TOKEN_ISSUED_KEY);
      }
      
      return issued ? parseInt(issued, 10) : null;
    } catch (error) {
      console.error('Error retrieving token issued:', error);
      return null;
    }
  }

  /**
   * Check if token is expired - DEPRECATED: Use backend validation instead
   * This is kept for backward compatibility but should not be used for authentication decisions
   */
  async isTokenExpired(): Promise<boolean> {
    console.warn('isTokenExpired is deprecated. Use backend validation via /api/v1/users/me instead.');
    const expiry = await this.getTokenExpiry();
    if (!expiry) return true;

    const currentTime = Math.floor(Date.now() / 1000);
    return expiry <= currentTime + 300; // 5 minutes buffer
  }

  /**
   * Check if token will expire soon - DEPRECATED: Use backend validation instead
   * This is kept for backward compatibility but should not be used for authentication decisions
   */
  async isTokenExpiringSoon(): Promise<boolean> {
    console.warn('isTokenExpiringSoon is deprecated. Use backend validation via /api/v1/users/me instead.');
    const expiry = await this.getTokenExpiry();
    if (!expiry) return true;

    const currentTime = Math.floor(Date.now() / 1000);
    return expiry <= currentTime + 900; // 15 minutes
  }

  /**
   * Clear all stored tokens
   */
  async clearTokens(): Promise<void> {
    try {
      if (this.isSecureStorageAvailable) {
        await SecureStore.deleteItemAsync(ACCESS_TOKEN_KEY);
        await SecureStore.deleteItemAsync(REFRESH_TOKEN_KEY);
        await SecureStore.deleteItemAsync(TOKEN_EXPIRY_KEY);
        await SecureStore.deleteItemAsync(TOKEN_ISSUED_KEY);
      } else {
        await AsyncStorage.removeItem(ACCESS_TOKEN_KEY);
        await AsyncStorage.removeItem(REFRESH_TOKEN_KEY);
        await AsyncStorage.removeItem(TOKEN_EXPIRY_KEY);
        await AsyncStorage.removeItem(TOKEN_ISSUED_KEY);
      }
    } catch (error) {
      console.error('Error clearing tokens:', error);
      throw new Error('Failed to clear authentication tokens');
    }
  }

  /**
   * Decode JWT token without verification (for client-side info only)
   */
  decodeToken(token: string): DecodedToken | null {
    try {
      const base64Url = token.split('.')[1];
      const base64 = base64Url.replace(/-/g, '+').replace(/_/g, '/');
      const jsonPayload = decodeURIComponent(
        atob(base64)
          .split('')
          .map(c => '%' + ('00' + c.charCodeAt(0).toString(16)).slice(-2))
          .join('')
      );
      return JSON.parse(jsonPayload);
    } catch (error) {
      console.error('Error decoding token:', error);
      return null;
    }
  }

  /**
   * Get user info from stored access token
   */
  async getUserInfo(): Promise<DecodedToken | null> {
    const accessToken = await this.getAccessToken();
    if (!accessToken) return null;

    return this.decodeToken(accessToken);
  }
}

// Singleton instance
export const tokenService = new TokenService();