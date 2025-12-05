import { MetricBatchCreate, metricService } from './metric.service';
import { networkService, NetworkService } from './network.service';
import { offlineStorageService, OfflineStorageService } from './offline-storage.service';
import { QueueItem, queueService, QueueService } from './queue.service';

export interface SyncResult {
  success: boolean;
  syncedItems: number;
  failedItems: number;
  totalItems: number;
  errors: string[];
  duration: number;
}

export interface SyncStatus {
  isSyncing: boolean;
  lastSync: number;
  lastSuccess: number;
  totalSynced: number;
  totalFailed: number;
  currentProgress?: number;
  currentOperation?: string;
}

export interface SyncConfig {
  batchSize: number;
  syncInterval: number;
  maxConcurrentBatches: number;
  retryOnFailure: boolean;
  backgroundSync: boolean;
}

export interface SyncEvent {
  type: 'start' | 'progress' | 'complete' | 'error';
  data?: SyncResult | number | Error;
  timestamp: number;
}

// Default synchronization configuration
const DEFAULT_SYNC_CONFIG: SyncConfig = {
  batchSize: 50,
  syncInterval: 30000, // 30 seconds
  maxConcurrentBatches: 3,
  retryOnFailure: true,
  backgroundSync: true
};

/**
 * Synchronization service for background data synchronization
 * Handles processing of queued items and coordination with offline storage
 */
export class SynchronizationService {
  private isRunning = false;
  private syncInterval: number | null = null;
  private config: SyncConfig = DEFAULT_SYNC_CONFIG;
  private status: SyncStatus = {
    isSyncing: false,
    lastSync: 0,
    lastSuccess: 0,
    totalSynced: 0,
    totalFailed: 0
  };
  
  private listeners: Map<string, Function> = new Map();
  private networkManager: NetworkService;
  private queueManager: QueueService;
  private storageService: OfflineStorageService;

  constructor() {
    this.networkManager = networkService;
    this.queueManager = queueService;
    this.storageService = offlineStorageService;
  }

  /**
   * Initialize synchronization service
   */
  async initialize(): Promise<void> {
    try {
      // Initialize dependencies
      await this.networkManager.initialize();
      await this.queueManager.initialize();
      await this.storageService.initialize();

      // Set up network change listener for automatic sync
      this.networkManager.addListener((state) => {
        if (state.isOnline && this.config.backgroundSync) {
          this.startSync().catch(error => {
            console.error('Auto-sync failed on network restoration:', error);
          });
        }
      });

      // Start background sync if enabled
      if (this.config.backgroundSync) {
        this.startBackgroundSync();
      }

      console.log('Synchronization service initialized');
    } catch (error) {
      console.error('Failed to initialize synchronization service:', error);
      throw error;
    }
  }

  /**
   * Start synchronization process
   */
  async startSync(): Promise<SyncResult> {
    if (this.status.isSyncing) {
      throw new Error('Sync already in progress');
    }

    const startTime = Date.now();
    this.status.isSyncing = true;
    this.status.lastSync = startTime;
    this.status.currentProgress = 0;
    this.status.currentOperation = 'Starting synchronization';

    this.emitEvent('start', { timestamp: startTime });

    try {
      // Check network connectivity
      if (!this.networkManager.isOnline()) {
        throw new Error('Network unavailable for synchronization');
      }

      // Get queue status
      const queueStatus = await this.queueManager.getStatus();
      if (queueStatus.pendingItems === 0 && queueStatus.failedItems === 0) {
        return this.createSuccessResult(0, 0, Date.now() - startTime);
      }

      // Process queue items
      const result = await this.processQueue();
      
      // Update sync statistics
      this.status.totalSynced += result.syncedItems;
      this.status.totalFailed += result.failedItems;
      
      if (result.success) {
        this.status.lastSuccess = Date.now();
      }

      this.emitEvent('complete', result);

      return result;
    } catch (error) {
      const errorResult = this.createErrorResult(error, Date.now() - startTime);
      this.emitEvent('error', error);
      throw error;
    } finally {
      this.status.isSyncing = false;
      this.status.currentProgress = undefined;
      this.status.currentOperation = undefined;
    }
  }

  /**
   * Stop synchronization process
   */
  stopSync(): void {
    this.status.isSyncing = false;
    this.status.currentProgress = undefined;
    this.status.currentOperation = undefined;
    
    if (this.syncInterval) {
      clearInterval(this.syncInterval);
      this.syncInterval = null;
    }
  }

  /**
   * Force immediate synchronization
   */
  async forceSync(): Promise<SyncResult> {
    return this.startSync();
  }

  /**
   * Get current synchronization status
   */
  getSyncStatus(): SyncStatus {
    return { ...this.status };
  }

  /**
   * Update synchronization configuration
   */
  setConfig(config: Partial<SyncConfig>): void {
    this.config = { ...this.config, ...config };
    
    // Restart background sync if configuration changed
    if (this.syncInterval) {
      this.stopSync();
      if (this.config.backgroundSync) {
        this.startBackgroundSync();
      }
    }
  }

  /**
   * Get current configuration
   */
  getConfig(): SyncConfig {
    return { ...this.config };
  }

  /**
   * Add event listener for sync events
   */
  onSyncStart(callback: () => void): () => void {
    const id = this.generateListenerId();
    this.listeners.set(id, callback);
    return () => this.listeners.delete(id);
  }

  /**
   * Add progress listener
   */
  onSyncProgress(callback: (progress: number) => void): () => void {
    const id = this.generateListenerId();
    this.listeners.set(id, callback);
    return () => this.listeners.delete(id);
  }

  /**
   * Add completion listener
   */
  onSyncComplete(callback: (result: SyncResult) => void): () => void {
    const id = this.generateListenerId();
    this.listeners.set(id, callback);
    return () => this.listeners.delete(id);
  }

  /**
   * Add error listener
   */
  onSyncError(callback: (error: Error) => void): () => void {
    const id = this.generateListenerId();
    this.listeners.set(id, callback);
    return () => this.listeners.delete(id);
  }

  /**
   * Clean up resources
   */
  destroy(): void {
    this.stopSync();
    this.listeners.clear();
    
    // Remove network listener
    this.networkManager.removeAllListeners();
  }

  /**
   * Process queued items for synchronization
   */
  private async processQueue(): Promise<SyncResult> {
    const startTime = Date.now();
    let syncedItems = 0;
    let failedItems = 0;
    const errors: string[] = [];

    // Get items to process
    const itemsToProcess = await this.queueManager.dequeue(this.config.batchSize);
    
    if (itemsToProcess.length === 0) {
      return this.createSuccessResult(0, 0, Date.now() - startTime);
    }

    this.status.currentOperation = `Processing ${itemsToProcess.length} items`;

    // Process items in batches
    const batchPromises = itemsToProcess.map(async (item) => {
      try {
        await this.processQueueItem(item);
        syncedItems++;
        
        // Update progress
        const progress = Math.round((syncedItems + failedItems) / itemsToProcess.length * 100);
        this.status.currentProgress = progress;
        this.emitEvent('progress', progress);
      } catch (error) {
        failedItems++;
        const errorMessage = error instanceof Error ? error.message : 'Unknown error';
        errors.push(`Item ${item.id}: ${errorMessage}`);
        
        // Update item status
        await this.queueManager.updateItem(item.id, {
          status: 'failed',
          error: errorMessage
        });
      }
    });

    // Process with concurrency limit
    const batchSize = Math.min(this.config.maxConcurrentBatches, batchPromises.length);
    for (let i = 0; i < batchPromises.length; i += batchSize) {
      const batch = batchPromises.slice(i, i + batchSize);
      await Promise.all(batch);
    }

    // Clear completed items
    await this.queueManager.clearCompleted();

    const duration = Date.now() - startTime;
    
    return {
      success: failedItems === 0,
      syncedItems,
      failedItems,
      totalItems: itemsToProcess.length,
      errors,
      duration
    };
  }

  /**
   * Process individual queue item
   */
  private async processQueueItem(item: QueueItem): Promise<void> {
    switch (item.type) {
      case 'metric_batch':
        await this.processMetricBatch(item);
        break;
      case 'issue':
        await this.processIssue(item);
        break;
      case 'device_event':
        await this.processDeviceEvent(item);
        break;
      default:
        throw new Error(`Unknown queue item type: ${item.type}`);
    }

    // Mark item as completed
    await this.queueManager.updateItem(item.id, {
      status: 'completed'
    });
  }

  /**
   * Process metric batch queue item
   */
  private async processMetricBatch(item: QueueItem): Promise<void> {
    const payload = item.payload;
    
    if (!payload || !Array.isArray(payload.metrics) || payload.metrics.length === 0) {
      throw new Error('Invalid metric batch payload');
    }

    // Use the new metric batch format (no device_id needed - uses JWT authentication)
    const metricBatch: MetricBatchCreate = {
      metrics: payload.metrics
    };
    await metricService.createMetricsBatch(metricBatch);

    // Update offline storage status if metric IDs are provided
    if (item.metadata?.metricIds) {
      await this.storageService.markMetricsAsSynced(item.metadata.metricIds);
    }
  }

  /**
   * Process issue queue item
   */
  private async processIssue(item: QueueItem): Promise<void> {
    // TODO: Implement issue processing when issue service is available
    console.log('Processing issue:', item.payload);
    throw new Error('Issue processing not yet implemented');
  }

  /**
   * Process device event queue item
   */
  private async processDeviceEvent(item: QueueItem): Promise<void> {
    // TODO: Implement device event processing when device service is available
    console.log('Processing device event:', item.payload);
    throw new Error('Device event processing not yet implemented');
  }

  /**
   * Start background synchronization
   */
  private startBackgroundSync(): void {
    if (this.syncInterval) {
      clearInterval(this.syncInterval);
    }

    this.syncInterval = setInterval(async () => {
      if (this.networkManager.isOnline() && !this.status.isSyncing) {
        try {
          await this.startSync();
        } catch (error) {
          console.error('Background sync failed:', error);
        }
      }
    }, this.config.syncInterval) as unknown as number;
  }

  /**
   * Create success result
   */
  private createSuccessResult(syncedItems: number, failedItems: number, duration: number): SyncResult {
    return {
      success: failedItems === 0,
      syncedItems,
      failedItems,
      totalItems: syncedItems + failedItems,
      errors: [],
      duration
    };
  }

  /**
   * Create error result
   */
  private createErrorResult(error: unknown, duration: number): SyncResult {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred';
    return {
      success: false,
      syncedItems: 0,
      failedItems: 0,
      totalItems: 0,
      errors: [errorMessage],
      duration
    };
  }

  /**
   * Emit sync event to listeners
   */
  private emitEvent(type: SyncEvent['type'], data?: any): void {
    const event: SyncEvent = {
      type,
      data,
      timestamp: Date.now()
    };

    this.listeners.forEach(callback => {
      try {
        switch (type) {
          case 'start':
            if (typeof callback === 'function') callback();
            break;
          case 'progress':
            if (typeof callback === 'function') callback(data);
            break;
          case 'complete':
            if (typeof callback === 'function') callback(data);
            break;
          case 'error':
            if (typeof callback === 'function') callback(data);
            break;
        }
      } catch (error) {
        console.error('Error in sync event listener:', error);
      }
    });
  }

  /**
   * Generate unique listener ID
   */
  private generateListenerId(): string {
    return `listener_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// Singleton instance
export const synchronizationService = new SynchronizationService();