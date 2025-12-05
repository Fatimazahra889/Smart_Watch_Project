// Token Manager Service - Handles token validation callbacks
// This service breaks the circular dependency between api.ts and auth.service.ts

export interface TokenValidationCallbacks {
  validateToken: () => Promise<boolean>;
  logout: () => Promise<void>;
  getAccessToken: () => Promise<string | null>;
}

class TokenManagerService {
  private callbacks: TokenValidationCallbacks | null = null;

  /**
   * Register token validation callbacks from auth service
   */
  registerCallbacks(callbacks: TokenValidationCallbacks): void {
    this.callbacks = callbacks;
  }

  /**
   * Validate token using registered callbacks
   */
  async validateToken(): Promise<boolean> {
    if (!this.callbacks) {
      console.warn('Token validation callbacks not registered');
      return false;
    }
    return this.callbacks.validateToken();
  }

  /**
   * Logout using registered callbacks
   */
  async logout(): Promise<void> {
    if (!this.callbacks) {
      console.warn('Logout callbacks not registered');
      return;
    }
    await this.callbacks.logout();
  }

  /**
   * Get access token using registered callbacks
   */
  async getAccessToken(): Promise<string | null> {
    if (!this.callbacks) {
      console.warn('Get access token callbacks not registered');
      return null;
    }
    return this.callbacks.getAccessToken();
  }

  /**
   * Check if callbacks are registered
   */
  isInitialized(): boolean {
    return this.callbacks !== null;
  }
}

// Singleton instance
export const tokenManagerService = new TokenManagerService();