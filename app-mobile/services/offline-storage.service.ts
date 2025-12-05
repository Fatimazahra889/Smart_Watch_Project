import AsyncStorage from '@react-native-async-storage/async-storage';
import { MetricCreate } from './metric.service';
import { networkService, NetworkService } from './network.service';
import { queueService, QueueService } from './queue.service';
import { deviceService } from './device.service';

export interface StoredMetric {
  id: string;
  metric: MetricCreate;
  storedAt: number;
  syncStatus: 'pending' | 'synced' | 'failed';
  syncAttempts: number;
  lastSyncAttempt?: number;
  syncError?: string;
  deviceId?: string;
  userId?: string;
}

export interface StorageStats {
  totalMetrics: number;
  pendingSync: number;
  storageSize: number;
  oldestMetric?: number;
  newestMetric?: number;
}

export interface SyncResult {
  success: boolean;
  syncedItems: number;
  failedItems: number;
  totalItems: number;
  errors: string[];
  duration: number;
}

// Storage keys
const OFFLINE_METRICS_KEY = 'offline_metrics';
const STORAGE_STATS_KEY = 'storage_stats';

/**
 * Offline storage service for health metrics
 * Provides persistent storage and synchronization capabilities
 */
export class OfflineStorageService {
  private networkManager: NetworkService;
  private queueManager: QueueService;
  private metrics: StoredMetric[] = [];
  private isInitialized = false;

  constructor() {
    this.networkManager = networkService;
    this.queueManager = queueService;
  }

  /**
   * Initialize storage service
   */
  async initialize(): Promise<void> {
    if (this.isInitialized) return;

    try {
      // Initialize dependencies
      await this.networkManager.initialize();
      await this.queueManager.initialize();

      // Load stored metrics
      const storedMetrics = await AsyncStorage.getItem(OFFLINE_METRICS_KEY);
      if (storedMetrics) {
        this.metrics = JSON.parse(storedMetrics);
      }

      this.isInitialized = true;
      console.log(`Offline storage initialized with ${this.metrics.length} metrics`);
    } catch (error) {
      console.error('Failed to initialize offline storage service:', error);
      throw error;
    }
  }

  /**
   * Store metrics locally with FIFO management
   * When storage is full, removes oldest metrics to make space
   */
  async storeMetrics(metrics: MetricCreate[]): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    if (metrics.length === 0) return;

    // Get current device ID for offline storage
    const currentDevice = deviceService.getCurrentDevice();
    const deviceId = currentDevice?.id;

    const storedMetrics: StoredMetric[] = metrics.map(metric => ({
      id: this.generateId(),
      metric,
      storedAt: Date.now(),
      syncStatus: 'pending',
      syncAttempts: 0,
      deviceId
    }));

    // Check if we need to make space for new metrics
    const maxMetrics = 1000; // Maximum number of metrics to store locally
    if (this.metrics.length + storedMetrics.length > maxMetrics) {
      // Calculate how many metrics to remove
      const metricsToRemove = Math.max(1, this.metrics.length + storedMetrics.length - maxMetrics);
      
      // Remove oldest metrics (sorted by storedAt ascending)
      const sortedMetrics = [...this.metrics].sort((a, b) => a.storedAt - b.storedAt);
      const metricsToKeep = sortedMetrics.slice(metricsToRemove);
      
      // Rebuild metrics array maintaining order but removing oldest metrics
      this.metrics = metricsToKeep;
      
      console.log(`FIFO storage management: Removed ${metricsToRemove} oldest metrics to make space for ${storedMetrics.length} new metrics`);
    }

    // Add to local storage
    this.metrics.push(...storedMetrics);
    await this.persistMetrics();

    // Add to sync queue if online
    if (this.networkManager.isOnline()) {
      try {
        await this.queueManager.enqueue({
          type: 'metric_batch',
          payload: {
            metrics: storedMetrics.map(sm => sm.metric),
            device_id: deviceId
          },
          priority: 'normal',
          maxAttempts: 5
        });
      } catch (error) {
        // Queue might be full, but we've stored metrics locally
        console.warn('Failed to add metrics to sync queue, but stored locally:', error);
      }
    }

    console.log(`Stored ${metrics.length} metrics offline${deviceId ? ` for device ${deviceId}` : ''}`);
  }

  /**
   * Get stored metrics with pagination
   */
  async getStoredMetrics(limit?: number, offset?: number): Promise<StoredMetric[]> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    let result = [...this.metrics].sort((a, b) => b.storedAt - a.storedAt);

    if (offset !== undefined) {
      result = result.slice(offset);
    }

    if (limit !== undefined) {
      result = result.slice(0, limit);
    }

    return result;
  }

  /**
   * Get pending metrics for synchronization
   */
  async getPendingMetrics(): Promise<StoredMetric[]> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    return this.metrics.filter(metric => metric.syncStatus === 'pending');
  }

  /**
   * Mark metrics as successfully synced
   */
  async markMetricsAsSynced(metricIds: string[]): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    let updatedCount = 0;
    for (const metric of this.metrics) {
      if (metricIds.includes(metric.id)) {
        metric.syncStatus = 'synced';
        metric.lastSyncAttempt = Date.now();
        updatedCount++;
      }
    }

    if (updatedCount > 0) {
      await this.persistMetrics();
      console.log(`Marked ${updatedCount} metrics as synced`);
    }
  }

  /**
   * Mark metrics as failed with error
   */
  async markMetricsAsFailed(metricIds: string[], error: string): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    let updatedCount = 0;
    for (const metric of this.metrics) {
      if (metricIds.includes(metric.id)) {
        metric.syncStatus = 'failed';
        metric.syncError = error;
        metric.lastSyncAttempt = Date.now();
        metric.syncAttempts += 1;
        updatedCount++;
      }
    }

    if (updatedCount > 0) {
      await this.persistMetrics();
      console.log(`Marked ${updatedCount} metrics as failed: ${error}`);
    }
  }

  /**
   * Clear successfully synced metrics
   */
  async clearSyncedMetrics(): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const initialLength = this.metrics.length;
    this.metrics = this.metrics.filter(metric => metric.syncStatus !== 'synced');
    
    if (this.metrics.length < initialLength) {
      await this.persistMetrics();
      console.log(`Cleared ${initialLength - this.metrics.length} synced metrics`);
    }
  }

  /**
   * Clear all metrics (use with caution)
   */
  async clearAllMetrics(): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    this.metrics = [];
    await this.persistMetrics();
    console.log('All metrics cleared from offline storage');
  }

  /**
   * Get storage statistics
   */
  async getStorageStats(): Promise<StorageStats> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const pendingSync = this.metrics.filter(m => m.syncStatus === 'pending').length;
    const timestamps = this.metrics.map(m => m.storedAt);

    return {
      totalMetrics: this.metrics.length,
      pendingSync,
      storageSize: await this.calculateStorageSize(),
      oldestMetric: timestamps.length > 0 ? Math.min(...timestamps) : undefined,
      newestMetric: timestamps.length > 0 ? Math.max(...timestamps) : undefined
    };
  }

  /**
   * Get storage usage in bytes
   */
  async getStorageUsage(): Promise<number> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    return this.calculateStorageSize();
  }

  /**
   * Clean up old data with authentication-based retention policy
   * @param isAuthenticated Whether the current user is authenticated
   * @param maxAge Optional override for maximum age in milliseconds (defaults to 30 days for authenticated, 7 days for unauthenticated)
   */
  async cleanupOldData(isAuthenticated: boolean, maxAge?: number): Promise<void>;
  
  /**
   * Clean up old data with specified maximum age (backward compatibility)
   * @param maxAge Maximum age in milliseconds (defaults to 30 days)
   */
  async cleanupOldData(maxAge: number): Promise<void>;
  
  /**
   * Clean up old data with default retention (7 days for unauthenticated users)
   */
  async cleanupOldData(): Promise<void>;
  
  async cleanupOldData(arg1?: boolean | number, arg2?: number): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    let isAuthenticated: boolean;
    let maxAge: number | undefined;

    // Handle different parameter patterns for backward compatibility
    if (typeof arg1 === 'boolean') {
      isAuthenticated = arg1;
      maxAge = arg2;
    } else if (typeof arg1 === 'number') {
      // Original signature: only maxAge provided
      isAuthenticated = false; // Default to unauthenticated for backward compatibility
      maxAge = arg1;
    } else {
      // No parameters provided - default to unauthenticated
      isAuthenticated = false;
      maxAge = undefined;
    }

    // Calculate retention period based on authentication status
    const retentionPeriod = maxAge ?? (isAuthenticated
      ? 30 * 24 * 60 * 60 * 1000 // 30 days for authenticated users
      : 7 * 24 * 60 * 60 * 1000  // 7 days for unauthenticated users
    );

    const cutoffTime = Date.now() - retentionPeriod;
    const initialLength = this.metrics.length;
    
    this.metrics = this.metrics.filter(metric => {
      // Keep pending metrics regardless of age
      if (metric.syncStatus === 'pending') return true;
      
      // Remove old synced/failed metrics based on retention policy
      return metric.storedAt > cutoffTime;
    });

    if (this.metrics.length < initialLength) {
      await this.persistMetrics();
      console.log(`Cleaned up ${initialLength - this.metrics.length} old metrics (${isAuthenticated ? 'authenticated' : 'unauthenticated'} retention: ${retentionPeriod / (24 * 60 * 60 * 1000)} days)`);
    }
  }

  /**
   * Synchronize pending metrics
   */
  async syncPendingMetrics(): Promise<SyncResult> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const startTime = Date.now();
    const pendingMetrics = await this.getPendingMetrics();
    
    if (pendingMetrics.length === 0) {
      return {
        success: true,
        syncedItems: 0,
        failedItems: 0,
        totalItems: 0,
        errors: [],
        duration: 0
      };
    }

    if (!this.networkManager.isOnline()) {
      return {
        success: false,
        syncedItems: 0,
        failedItems: pendingMetrics.length,
        totalItems: pendingMetrics.length,
        errors: ['Network unavailable for synchronization'],
        duration: Date.now() - startTime
      };
    }

    // Add to sync queue for processing
    const metricIds = pendingMetrics.map(m => m.id);
    const metricsBatch = pendingMetrics.map(m => m.metric);

    try {
      await this.queueManager.enqueue({
        type: 'metric_batch',
        payload: { metrics: metricsBatch },
        priority: 'normal',
        maxAttempts: 5,
        metadata: { metricIds }
      });

      return {
        success: true,
        syncedItems: 0, // Will be updated when queue processes
        failedItems: 0,
        totalItems: pendingMetrics.length,
        errors: [],
        duration: Date.now() - startTime
      };
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      return {
        success: false,
        syncedItems: 0,
        failedItems: pendingMetrics.length,
        totalItems: pendingMetrics.length,
        errors: [errorMessage],
        duration: Date.now() - startTime
      };
    }
  }

  /**
   * Check if device is online
   */
  isOnline(): boolean {
    return this.networkManager.isOnline();
  }

  /**
   * Get queue status
   */
  async getQueueStatus() {
    if (!this.isInitialized) {
      await this.initialize();
    }

    return this.queueManager.getStatus();
  }

  /**
   * Set network manager (for testing/dependency injection)
   */
  setNetworkManager(networkManager: NetworkService): void {
    this.networkManager = networkManager;
  }

  /**
   * Set queue manager (for testing/dependency injection)
   */
  setQueueManager(queueManager: QueueService): void {
    this.queueManager = queueManager;
  }

  /**
   * Persist metrics to storage
   */
  private async persistMetrics(): Promise<void> {
    try {
      await AsyncStorage.setItem(OFFLINE_METRICS_KEY, JSON.stringify(this.metrics));
    } catch (error) {
      console.error('Failed to persist metrics:', error);
      throw error;
    }
  }

  /**
   * Calculate approximate storage size
   */
  private async calculateStorageSize(): Promise<number> {
    try {
      const metricsJson = JSON.stringify(this.metrics);
      return new Blob([metricsJson]).size;
    } catch (error) {
      console.error('Error calculating storage size:', error);
      return 0;
    }
  }

  /**
   * Generate unique ID for stored metrics
   */
  private generateId(): string {
    return `metric_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// Singleton instance
export const offlineStorageService = new OfflineStorageService();