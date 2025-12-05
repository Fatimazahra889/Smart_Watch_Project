
export interface NetworkState {
  isOnline: boolean;
  connectionType: 'wifi' | 'cellular' | 'ethernet' | 'none';
  effectiveType?: 'slow-2g' | '2g' | '3g' | '4g';
  lastChecked: number;
}

export type ConnectionType = 'wifi' | 'cellular' | 'ethernet' | 'none';
export type NetworkChangeCallback = (state: NetworkState) => void;

/**
 * Network connectivity monitoring service
 * Provides real-time network state detection and event broadcasting
 */
export class NetworkService {
  private state: NetworkState = {
    isOnline: true,
    connectionType: 'none',
    lastChecked: Date.now()
  };
  
  private listeners: Set<NetworkChangeCallback> = new Set();
  private unsubscribe: (() => void) | null = null;

  /**
   * Initialize network monitoring
   */
  async initialize(): Promise<void> {
    try {
      // For React Native, we would use NetInfo from '@react-native-async-storage/async-storage'
      // For now, we'll implement a basic version that works in Expo/React Native environments
      
      // Set initial state
      await this.updateNetworkState();
      
      // Set up periodic checks (every 30 seconds)
      setInterval(() => {
        this.updateNetworkState();
      }, 30000);
      
      console.log('Network service initialized');
    } catch (error) {
      console.error('Failed to initialize network service:', error);
      throw error;
    }
  }

  /**
   * Clean up network monitoring
   */
  destroy(): void {
    if (this.unsubscribe) {
      this.unsubscribe();
      this.unsubscribe = null;
    }
    this.listeners.clear();
  }

  /**
   * Check if device is online
   */
  isOnline(): boolean {
    return this.state.isOnline;
  }

  /**
   * Get current connection type
   */
  getConnectionType(): ConnectionType {
    return this.state.connectionType;
  }

  /**
   * Get complete network state
   */
  getNetworkState(): NetworkState {
    return { ...this.state };
  }

  /**
   * Add network state change listener
   */
  addListener(callback: NetworkChangeCallback): () => void {
    this.listeners.add(callback);
    
    // Return unsubscribe function
    return () => {
      this.listeners.delete(callback);
    };
  }

  /**
   * Remove specific listener
   */
  removeListener(callback: NetworkChangeCallback): void {
    this.listeners.delete(callback);
  }

  /**
   * Remove all listeners
   */
  removeAllListeners(): void {
    this.listeners.clear();
  }

  /**
   * Update network state and notify listeners if changed
   */
  private async updateNetworkState(): Promise<void> {
    const previousState = { ...this.state };
    
    try {
      // In a real React Native app, we would use:
      // const netInfo = await NetInfo.fetch();
      // this.state.isOnline = netInfo.isConnected ?? false;
      // this.state.connectionType = this.mapConnectionType(netInfo.type);
      
      // For now, we'll simulate network detection
      // In production, this should be replaced with actual NetInfo implementation
      const isOnline = await this.checkConnectivity();
      const connectionType = await this.detectConnectionType();
      
      const newState: NetworkState = {
        isOnline,
        connectionType,
        lastChecked: Date.now()
      };
      
      // Update state
      this.state = newState;
      
      // Notify listeners if state changed
      if (this.hasStateChanged(previousState, newState)) {
        this.notifyListeners(newState);
      }
    } catch (error) {
      console.error('Error updating network state:', error);
      
      // Fallback to offline state on error
      const fallbackState: NetworkState = {
        isOnline: false,
        connectionType: 'none',
        lastChecked: Date.now()
      };
      
      if (this.hasStateChanged(previousState, fallbackState)) {
        this.state = fallbackState;
        this.notifyListeners(fallbackState);
      }
    }
  }

  /**
   * Check connectivity by attempting to reach a reliable endpoint
   */
  private async checkConnectivity(): Promise<boolean> {
    try {
      // Try to fetch a small resource from a reliable CDN
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000);
      
      const response = await fetch('https://www.google.com/favicon.ico', {
        method: 'HEAD',
        signal: controller.signal,
        cache: 'no-cache'
      });
      
      clearTimeout(timeoutId);
      return response.ok;
    } catch (error) {
      return false;
    }
  }

  /**
   * Detect connection type (simplified for now)
   */
  private async detectConnectionType(): Promise<ConnectionType> {
    // In a real React Native app, we would use NetInfo to get connection type
    // For now, we'll return a default value
    return 'wifi'; // Default assumption
  }

  /**
   * Map NetInfo connection type to our ConnectionType
   */
  private mapConnectionType(netInfoType: string): ConnectionType {
    switch (netInfoType) {
      case 'wifi':
        return 'wifi';
      case 'cellular':
        return 'cellular';
      case 'ethernet':
        return 'ethernet';
      case 'none':
      default:
        return 'none';
    }
  }

  /**
   * Check if network state has changed significantly
   */
  private hasStateChanged(prev: NetworkState, current: NetworkState): boolean {
    return (
      prev.isOnline !== current.isOnline ||
      prev.connectionType !== current.connectionType
    );
  }

  /**
   * Notify all listeners of network state change
   */
  private notifyListeners(state: NetworkState): void {
    this.listeners.forEach(callback => {
      try {
        callback(state);
      } catch (error) {
        console.error('Error in network state listener:', error);
      }
    });
  }

  /**
   * Force a network state check
   */
  async forceCheck(): Promise<NetworkState> {
    await this.updateNetworkState();
    return this.getNetworkState();
  }

  /**
   * Get network quality indicator (simplified)
   */
  getNetworkQuality(): 'excellent' | 'good' | 'fair' | 'poor' {
    if (!this.state.isOnline) {
      return 'poor';
    }

    switch (this.state.connectionType) {
      case 'wifi':
        return 'excellent';
      case 'ethernet':
        return 'excellent';
      case 'cellular':
        return 'good';
      case 'none':
      default:
        return 'poor';
    }
  }
}

// Singleton instance
export const networkService = new NetworkService();