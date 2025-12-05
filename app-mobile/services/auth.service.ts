import { ApiError, apiService, setAuthToken, NetworkError } from './api';
import { tokenManagerService, TokenValidationCallbacks } from './token-manager.service';
import { DecodedToken, TokenPair, tokenService } from './token.service';

export interface Token {
  access_token: string;
  token_type: string;
}

export interface LoginCredentials {
  email: string;
  password: string;
}

export interface EmailVerificationRequest {
  email: string;
  verification_code: string;
}

export interface ResendCodeRequest {
  email: string;
}

export class AuthService {
  /**
   * Login with email and password using JWT authentication
   */
  async login(credentials: LoginCredentials): Promise<Token> {
    try {
      const token = await apiService.post<Token>('/api/v1/auth/login', credentials);

      console.log('Login successful, storing token', token);
      
      await this.setToken(token);
      
      return token;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Invalid email or password', 401, 'INVALID_CREDENTIALS');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Email not verified', 403, 'EMAIL_NOT_VERIFIED');
      }
      throw error;
    }
  }

  /**
   * Logout and clear stored tokens
   */
  async logout(): Promise<void> {
    try {
      // Call logout endpoint if available
      await apiService.post('/api/v1/auth/logout');
    } catch (error) {
      // Continue with logout even if endpoint fails
      console.warn('Logout endpoint call failed, continuing with client-side logout');
    }
    
    // Clear stored tokens
    await tokenService.clearTokens();
    
    // Clear API service token
    setAuthToken(null);
  }

  /**
   * Check if user is authenticated using token validation
   */
  async isAuthenticated(): Promise<boolean> {
    try {
      const accessToken = await tokenService.getAccessToken();
      if (!accessToken) {
        return false;
      }
      
      // For JWT tokens, we can validate locally first
      const userInfo = await tokenService.getUserInfo();
      if (!userInfo) {
        return false;
      }
      
      // Token exists and is valid locally
      return true;
    } catch (error) {
      console.error('Error checking authentication:', error);
      return false;
    }
  }

  /**
   * Get current user info from token
   */
  async getUserInfo(): Promise<DecodedToken | null> {
    return await tokenService.getUserInfo();
  }

  /**
   * Verify user email with verification code
   */
  async verifyEmail(request: EmailVerificationRequest): Promise<void> {
    try {
      await apiService.post('/api/v1/auth/verify-email', request);
    } catch (error) {
      if (error instanceof ApiError && error.status === 400) {
        throw new ApiError('Invalid verification code', 400, 'INVALID_VERIFICATION_CODE');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('User not found', 404, 'USER_NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Resend verification code
   */
  async resendVerificationCode(request: ResendCodeRequest): Promise<void> {
    try {
      await apiService.post('/api/v1/auth/resend-code', request);
    } catch (error) {
      // Always return success for security reasons (prevents email enumeration)
      console.log('Resend code request processed (security policy)');
    }
  }

  /**
   * Validate current token
   * For JWT tokens, we rely on local validation and let API calls fail naturally
   */
  async validateToken(): Promise<boolean> {
    try {
      const accessToken = await tokenService.getAccessToken();
      if (!accessToken) {
        return false;
      }
      
      // For JWT, we can validate locally
      const userInfo = await tokenService.getUserInfo();
      return !!userInfo;
    } catch (error) {
      console.error('Error validating token:', error);
      return false;
    }
  }

  /**
   * Get access token
   */
  async getAccessToken(): Promise<string | null> {
    return await tokenService.getAccessToken();
  }

  /**
   * Set token and store securely
   */
  private async setToken(token: Token): Promise<void> {
    // Validate required fields
    if (!token.access_token) {
      throw new Error('Access token is required');
    }

    console.log('Storing token:', {
      hasAccessToken: !!token.access_token,
      tokenType: token.token_type
    });

    // Create token pair for secure storage (JWT doesn't use refresh tokens)
    const tokenPair: TokenPair = {
      accessToken: token.access_token,
      refreshToken: '', // JWT doesn't use refresh tokens
      expiresAt: 0, // JWT expiration is handled by the token itself
      issuedAt: Math.floor(Date.now() / 1000),
    };

    // Store tokens securely
    await tokenService.storeTokens(tokenPair);

    // Update API service with new token
    setAuthToken(token.access_token);
  }

  /**
   * Initialize authentication state on app start
   */
  async initialize(): Promise<boolean> {
    try {
      // First, try to get token from storage
      const accessToken = await tokenService.getAccessToken();
      console.log('Token initialization - access token available:', !!accessToken);
      
      if (accessToken) {
        // Set token in API service
        setAuthToken(accessToken);
        console.log('Token set in API service for initialization');
        
        // Validate the token locally
        const isAuthenticated = await this.isAuthenticated();
        
        if (isAuthenticated) {
          console.log('Authentication initialized successfully');
          return true;
        } else {
          // Token exists but is invalid
          console.log('Token exists but validation failed');
          await this.logout();
          return false;
        }
      } else {
        // No token available
        console.log('No access token available for initialization');
        return false;
      }
    } catch (error) {
      console.error('Error initializing authentication:', error);
      return false;
    }
  }
}

// Singleton instance
export const authService = new AuthService();

// Register token validation callbacks with the token manager
const callbacks: TokenValidationCallbacks = {
  validateToken: () => authService.validateToken(),
  logout: () => authService.logout(),
  getAccessToken: () => authService.getAccessToken(),
};

tokenManagerService.registerCallbacks(callbacks);