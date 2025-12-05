// Base API service with error handling and configuration
import { tokenManagerService } from './token-manager.service';

export interface ApiConfig {
  baseURL: string;
  timeout: number;
  retryAttempts: number;
}

export interface ApiErrorResponse {
  detail?: {
    loc: (string | number)[];
    msg: string;
    type: string;
  }[] | string;
  message?: string;
}

// Custom error types
export class ApiError extends Error {
  constructor(
    message: string,
    public status: number,
    public code?: string,
    public details?: ApiErrorResponse
  ) {
    super(message);
    this.name = 'ApiError';
  }
}

export class NetworkError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'NetworkError';
  }
}

export class ValidationError extends Error {
  constructor(
    message: string,
    public details: ApiErrorResponse['detail'] = []
  ) {
    super(message);
    this.name = 'ValidationError';
  }
}

// Default configuration
const defaultConfig: ApiConfig = {
  baseURL: process.env.EXPO_PUBLIC_API_URL || 'http://localhost:8000',
  timeout: 60000,
  retryAttempts: 3
};

// Token storage (in a real app, use secure storage)
let authToken: string | null = null;

export const setAuthToken = (token: string | null) => {
  authToken = token;
};

export const getAuthToken = (): string | null => authToken;

/**
 * Get authentication headers - JWT Bearer tokens are supported
 */
export const getAuthHeaders = (): Record<string, string> => {
  const headers: Record<string, string> = {};
  
  if (authToken) {
    headers['Authorization'] = `Bearer ${authToken}`;
  } else {
    console.warn('JWT Bearer authentication requested but no auth token available - API calls may fail');
  }
  
  return headers;
};

/**
 * Check if authentication token is available in API service
 */
export const isAuthTokenAvailable = (): boolean => {
  return authToken !== null && authToken.trim() !== '';
};

// Main API service class
export class ApiService {
  private config: ApiConfig;

  constructor(config: Partial<ApiConfig> = {}) {
    this.config = { ...defaultConfig, ...config };
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {},
    retryCount = 0
  ): Promise<T> {
    const url = `${this.config.baseURL}${endpoint}`;

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      ...(options.headers as Record<string, string>),
    };

    // Add authentication headers (JWT Bearer only)
    const authHeaders = getAuthHeaders();
    Object.assign(headers, authHeaders);

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      console.log(options.method + " => " + url);
      if (options.body) {
        console.log("Body: ", options.body);
      }
      
      const response = await fetch(url, {
        ...options,
        headers,
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        let errorData: ApiErrorResponse | null = null;
        
        try {
          errorData = await response.json();
        } catch {
          // Ignore JSON parsing errors for error responses
        }

        // Handle specific HTTP status codes
        if (response.status === 401) {
          // Unauthorized - logout and throw
          if (tokenManagerService.isInitialized()) {
            await tokenManagerService.logout();
          }
          throw new ApiError('Unauthorized - Please login again', 401, 'UNAUTHORIZED', errorData || undefined);
        } else if (response.status === 403) {
          throw new ApiError('Forbidden - Insufficient permissions', 403, 'FORBIDDEN', errorData || undefined);
        } else if (response.status === 422) {
          throw new ValidationError('Validation failed', errorData?.detail);
        } else if (response.status >= 500) {
          throw new ApiError('Server error', response.status, 'SERVER_ERROR', errorData || undefined);
        } else {
          throw new ApiError(
            errorData?.message || `HTTP error ${response.status}`,
            response.status,
            'HTTP_ERROR',
            errorData || undefined
          );
        }
      }

      const data = await response.json();
      console.log("GOT: ", data);
      
      return data as T;

    } catch (error) {
      clearTimeout(timeoutId);

      // Handle network errors and retry logic
      if (error instanceof TypeError && error.message.includes('Network request failed')) {
        if (retryCount < this.config.retryAttempts) {
          // Exponential backoff
          await new Promise(resolve =>
            setTimeout(resolve, Math.pow(2, retryCount) * 1000)
          );
          return this.request<T>(endpoint, options, retryCount + 1);
        }
        throw new NetworkError('Network connection failed');
      }

      if (error instanceof Error && error.name === 'AbortError') {
        throw new NetworkError('Request timeout');
      }

      // Re-throw known error types
      if (error instanceof ApiError || error instanceof NetworkError || error instanceof ValidationError) {
        throw error;
      }

      // Wrap unknown errors
      throw new ApiError(
        error instanceof Error ? error.message : 'Unknown error occurred',
        0,
        'UNKNOWN_ERROR'
      );
    }
  }

  // HTTP method wrappers
  async get<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    return this.request<T>(endpoint, { ...options, method: 'GET' });
  }

  async post<T>(endpoint: string, data?: any, options: RequestInit = {}): Promise<T> {
    return this.request<T>(endpoint, {
      ...options,
      method: 'POST',
      body: data ? (typeof data === 'string' ? data : data instanceof URLSearchParams ? data.toString() : JSON.stringify(data)) : undefined,
    });
  }

  async put<T>(endpoint: string, data?: any, options: RequestInit = {}): Promise<T> {
    return this.request<T>(endpoint, {
      ...options,
      method: 'PUT',
      body: data ? JSON.stringify(data) : undefined,
    });
  }

  async delete<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    return this.request<T>(endpoint, { ...options, method: 'DELETE' });
  }

  /**
   * Upload file with form data
   */
  async upload<T>(endpoint: string, formData: FormData, options: RequestInit = {}): Promise<T> {
    const headers: Record<string, string> = {};
    
    // Add authentication headers (JWT Bearer only)
    const authHeaders = getAuthHeaders();
    Object.assign(headers, authHeaders);

    return this.request<T>(endpoint, {
      ...options,
      method: 'POST',
      headers,
      body: formData,
    });
  }
}

// Singleton instance
export const apiService = new ApiService();