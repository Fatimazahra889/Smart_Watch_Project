import { apiService, ApiError, NetworkError, ValidationError } from './api';

export interface User {
  id: string;
  email?: string;
  name?: string;
  is_admin?: boolean;
  created_at: string;
  updated_at: string;
  email_verified_at?: string;
  deleted_at?: string;
}

export interface UserCreate {
  email: string;
  name: string;
  password: string;
}

export interface UserUpdate {
  email?: string;
  name?: string;
  is_admin?: boolean;
  password?: string;
}

export interface EmailVerificationRequest {
  email: string;
  verification_code: string;
}

export interface ResendCodeRequest {
  email: string;
}

export class UserService {
  /**
   * Register a new user (use auth registration endpoint)
   */
  async registerUser(userData: UserCreate): Promise<any> {
    try {
      return await apiService.post<any>('/api/v1/auth/register', userData);
    } catch (error) {
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('User registration validation failed', error.details?.detail);
      }
      if (error instanceof ApiError && error.status === 409) {
        throw new ApiError('User already exists', 409, 'USER_ALREADY_EXISTS');
      }
      throw error;
    }
  }

  /**
   * Create a new user (legacy method - use registerUser instead)
   */
  async createUser(userData: UserCreate): Promise<any> {
    return this.registerUser(userData);
  }

  /**
   * Get all users (admin only)
   */
  async getUsers(skip: number = 0, limit: number = 100): Promise<User[]> {
    try {
      const params = new URLSearchParams({
        skip: skip.toString(),
        limit: limit.toString(),
      });
      
      return await apiService.get<User[]>(`/api/v1/users/?${params}`);
    } catch (error) {
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to view users', 403, 'FORBIDDEN');
      }
      throw error;
    }
  }

  /**
   * Get user by ID
   */
  async getUserById(userId: string): Promise<User> {
    try {
      return await apiService.get<User>(`/api/v1/users/${userId}`);
    } catch (error) {
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('User not found', 404, 'USER_NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Update user
   */
  async updateUser(userId: string, updates: UserUpdate): Promise<User> {
    try {
      return await apiService.put<User>(`/api/v1/users/${userId}`, updates);
    } catch (error) {
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('User not found', 404, 'USER_NOT_FOUND');
      }
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('User update validation failed', error.details?.detail);
      }
      throw error;
    }
  }

  /**
   * Delete user (soft delete)
   */
  async deleteUser(userId: string): Promise<User> {
    try {
      return await apiService.delete<User>(`/api/v1/users/${userId}`);
    } catch (error) {
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('User not found', 404, 'USER_NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Get current user (requires authentication)
   * Note: This endpoint is no longer used for token validation
   */
  async getCurrentUser(): Promise<User> {
    try {
      return await apiService.get<User>('/api/v1/users/me');
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('User not found', 404, 'USER_NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Update current user profile
   */
  async updateCurrentUser(updates: UserUpdate): Promise<User> {
    try {
      // This would typically use a /me endpoint
      // For now, we'll throw not implemented
      throw new ApiError('Update current user not implemented', 501, 'NOT_IMPLEMENTED');
    } catch (error) {
      throw error;
    }
  }
}

// Singleton instance
export const userService = new UserService();