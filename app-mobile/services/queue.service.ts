import AsyncStorage from '@react-native-async-storage/async-storage';

export interface QueueItem {
  id: string;
  type: 'metric_batch' | 'issue' | 'device_event';
  payload: any;
  timestamp: number;
  priority: 'high' | 'normal' | 'low';
  attempts: number;
  maxAttempts: number;
  lastAttempt?: number;
  status: 'pending' | 'processing' | 'failed' | 'completed';
  error?: string;
  metadata?: Record<string, any>;
}

export interface QueueStatus {
  totalItems: number;
  pendingItems: number;
  processingItems: number;
  failedItems: number;
  completedItems: number;
  lastProcessed: number;
}

export interface RetryConfig {
  maxAttempts: number;
  baseDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}

export interface QueueStorage {
  items: QueueItem[];
  lastProcessed: number;
  totalProcessed: number;
  totalFailed: number;
}

// Storage keys
const QUEUE_STORAGE_KEY = 'sync_queue';
const QUEUE_STATS_KEY = 'queue_stats';

// Default retry configuration
const DEFAULT_RETRY_CONFIG: RetryConfig = {
  maxAttempts: 5,
  baseDelay: 1000, // 1 second
  maxDelay: 300000, // 5 minutes
  backoffMultiplier: 2
};

/**
 * Queue management service with retry logic and persistence
 * Handles synchronization queue for offline operations
 */
export class QueueService {
  private queue: QueueItem[] = [];
  private maxSize: number = 1000;
  private retryConfig: RetryConfig = DEFAULT_RETRY_CONFIG;
  private isInitialized = false;

  /**
   * Initialize queue from persistent storage
   */
  async initialize(): Promise<void> {
    if (this.isInitialized) return;

    try {
      // Load queue from storage
      const storedQueue = await AsyncStorage.getItem(QUEUE_STORAGE_KEY);
      if (storedQueue) {
        const parsedQueue: QueueItem[] = JSON.parse(storedQueue);
        this.queue = parsedQueue.filter(item => item.status !== 'completed');
        
        // Clean up completed items from storage
        if (this.queue.length !== parsedQueue.length) {
          await this.persistQueue();
        }
      }

      this.isInitialized = true;
      console.log(`Queue service initialized with ${this.queue.length} items`);
    } catch (error) {
      console.error('Failed to initialize queue service:', error);
      throw error;
    }
  }

  /**
   * Add item to queue with FIFO management
   * When queue is full, removes oldest items to make space
   */
  async enqueue(itemData: Omit<QueueItem, 'id' | 'timestamp' | 'attempts' | 'status'>): Promise<string> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const item: QueueItem = {
      ...itemData,
      id: this.generateId(),
      timestamp: Date.now(),
      attempts: 0,
      status: 'pending'
    };

    // Check if we need to make space for new item
    if (this.queue.length >= this.maxSize) {
      // Calculate how many items to remove (at least 1)
      const itemsToRemove = Math.max(1, this.queue.length - this.maxSize + 1);
      
      // Remove oldest items (sorted by timestamp ascending)
      const sortedQueue = [...this.queue].sort((a, b) => a.timestamp - b.timestamp);
      const itemsToKeep = sortedQueue.slice(itemsToRemove);
      
      // Rebuild queue maintaining priority order but removing oldest items
      this.queue = itemsToKeep;
      
      console.log(`FIFO queue management: Removed ${itemsToRemove} oldest items to make space for new item`);
    }

    this.queue.push(item);
    await this.persistQueue();

    console.log(`Item ${item.id} added to queue (type: ${item.type}, priority: ${item.priority})`);
    return item.id;
  }

  /**
   * Get next items for processing
   */
  async dequeue(maxItems: number = 10): Promise<QueueItem[]> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    // Get pending items, sorted by priority and timestamp
    const pendingItems = this.queue
      .filter(item => item.status === 'pending')
      .sort((a, b) => {
        // Sort by priority first, then by timestamp
        const priorityOrder = { high: 0, normal: 1, low: 2 };
        if (priorityOrder[a.priority] !== priorityOrder[b.priority]) {
          return priorityOrder[a.priority] - priorityOrder[b.priority];
        }
        return a.timestamp - b.timestamp;
      })
      .slice(0, maxItems);

    // Mark items as processing
    for (const item of pendingItems) {
      item.status = 'processing';
      item.lastAttempt = Date.now();
      item.attempts += 1;
    }

    await this.persistQueue();
    return pendingItems;
  }

  /**
   * Update queue item
   */
  async updateItem(id: string, updates: Partial<QueueItem>): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const itemIndex = this.queue.findIndex(item => item.id === id);
    if (itemIndex === -1) {
      throw new Error(`Queue item ${id} not found`);
    }

    this.queue[itemIndex] = { ...this.queue[itemIndex], ...updates };
    await this.persistQueue();
  }

  /**
   * Remove item from queue
   */
  async removeItem(id: string): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const itemIndex = this.queue.findIndex(item => item.id === id);
    if (itemIndex === -1) {
      throw new Error(`Queue item ${id} not found`);
    }

    this.queue.splice(itemIndex, 1);
    await this.persistQueue();
  }

  /**
   * Get queue status
   */
  async getStatus(): Promise<QueueStatus> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const pendingItems = this.queue.filter(item => item.status === 'pending').length;
    const processingItems = this.queue.filter(item => item.status === 'processing').length;
    const failedItems = this.queue.filter(item => item.status === 'failed').length;
    const completedItems = this.queue.filter(item => item.status === 'completed').length;

    return {
      totalItems: this.queue.length,
      pendingItems,
      processingItems,
      failedItems,
      completedItems,
      lastProcessed: this.getLastProcessedTime()
    };
  }

  /**
   * Get items by status
   */
  async getItems(status?: QueueItem['status']): Promise<QueueItem[]> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    if (status) {
      return this.queue.filter(item => item.status === status);
    }
    return [...this.queue];
  }

  /**
   * Clear completed items from queue
   */
  async clearCompleted(): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const initialLength = this.queue.length;
    this.queue = this.queue.filter(item => item.status !== 'completed');
    
    if (this.queue.length < initialLength) {
      await this.persistQueue();
      console.log(`Cleared ${initialLength - this.queue.length} completed items from queue`);
    }
  }

  /**
   * Retry failed items
   */
  async retryFailed(maxAttempts?: number): Promise<void> {
    if (!this.isInitialized) {
      await this.initialize();
    }

    const failedItems = this.queue.filter(item => 
      item.status === 'failed' && 
      item.attempts < (maxAttempts || this.retryConfig.maxAttempts)
    );

    for (const item of failedItems) {
      item.status = 'pending';
      item.error = undefined;
    }

    if (failedItems.length > 0) {
      await this.persistQueue();
      console.log(`Retrying ${failedItems.length} failed items`);
    }
  }

  /**
   * Calculate retry delay for an item
   */
  calculateRetryDelay(attempts: number): number {
    const delay = this.retryConfig.baseDelay * Math.pow(this.retryConfig.backoffMultiplier, attempts - 1);
    return Math.min(delay, this.retryConfig.maxDelay);
  }

  /**
   * Check if item should be retried
   */
  shouldRetry(item: QueueItem): boolean {
    return item.attempts < item.maxAttempts && item.status === 'failed';
  }

  /**
   * Set maximum queue size
   */
  setMaxQueueSize(size: number): void {
    this.maxSize = Math.max(1, size);
  }

  /**
   * Set retry configuration
   */
  setRetryConfig(config: Partial<RetryConfig>): void {
    this.retryConfig = { ...this.retryConfig, ...config };
  }

  /**
   * Get current retry configuration
   */
  getRetryConfig(): RetryConfig {
    return { ...this.retryConfig };
  }

  /**
   * Get queue size
   */
  getQueueSize(): number {
    return this.queue.length;
  }

  /**
   * Get maximum queue size
   */
  getMaxQueueSize(): number {
    return this.maxSize;
  }

  /**
   * Check if queue is empty
   */
  isEmpty(): boolean {
    return this.queue.length === 0;
  }

  /**
   * Clear entire queue
   */
  async clearAll(): Promise<void> {
    this.queue = [];
    await this.persistQueue();
    console.log('Queue cleared');
  }

  /**
   * Persist queue to storage
   */
  private async persistQueue(): Promise<void> {
    try {
      await AsyncStorage.setItem(QUEUE_STORAGE_KEY, JSON.stringify(this.queue));
    } catch (error) {
      console.error('Failed to persist queue:', error);
      throw error;
    }
  }

  /**
   * Generate unique ID for queue items
   */
  private generateId(): string {
    return `queue_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Get last processed time from queue
   */
  private getLastProcessedTime(): number {
    const completedItems = this.queue.filter(item => item.status === 'completed');
    if (completedItems.length === 0) return 0;
    
    return Math.max(...completedItems.map(item => item.lastAttempt || item.timestamp));
  }
}

// Singleton instance
export const queueService = new QueueService();