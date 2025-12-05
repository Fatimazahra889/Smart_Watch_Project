import { ApiError, apiService, ValidationError } from './api';

export enum IssueSeverity {
  LOW = "low",
  MODERATE = "moderate",
  CRITICAL = "critical"
}

export interface Issue {
  id: string;
  user_id: string;
  issue_type?: string;
  description?: string;
  severity?: IssueSeverity;
  detected_at?: string;
  resolved?: boolean;
  created_at: string;
  updated_at: string;
  deleted_at?: string;
}

export interface IssueCreate {
  issue_type?: string;
  description?: string;
  severity?: IssueSeverity;
  detected_at?: string;
}

export interface IssueUpdate {
  issue_type?: string;
  description?: string;
  severity?: IssueSeverity;
  detected_at?: string;
  resolved?: boolean;
}

export interface IssueQueryParams {
  severity?: IssueSeverity;
  resolved?: boolean;
  start_date?: string;
  end_date?: string;
  limit?: number;
  offset?: number;
}

export class IssueService {
  private issuesCache: Map<string, Issue[]> = new Map();

  /**
   * Get user issues
   */
  async getUserIssues(userId: string, params: IssueQueryParams = {}): Promise<Issue[]> {
    const cacheKey = this.generateCacheKey(userId, params);
    
    // Check cache first
    if (this.issuesCache.has(cacheKey)) {
      return this.issuesCache.get(cacheKey)!;
    }

    try {
      const queryParams = new URLSearchParams();
      queryParams.append('user_id', userId);
      
      if (params.severity) queryParams.append('severity', params.severity);
      if (params.resolved !== undefined) queryParams.append('resolved', params.resolved.toString());
      if (params.start_date) queryParams.append('start_date', params.start_date);
      if (params.end_date) queryParams.append('end_date', params.end_date);
      if (params.limit) queryParams.append('limit', params.limit.toString());
      if (params.offset) queryParams.append('offset', params.offset.toString());

      const queryString = queryParams.toString();
      const endpoint = queryString ? `/api/v1/issues/?${queryString}` : '/api/v1/issues/';
      
      const response = await apiService.get<Issue[]>(endpoint);
      
      // Cache the results
      this.cacheIssues(cacheKey, response);
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access issues', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to access issues', 403, 'FORBIDDEN');
      }
      throw error;
    }
  }

  /**
   * Create a new issue
   */
  async createIssue(issueData: IssueCreate): Promise<Issue> {
    try {
      const response = await apiService.post<Issue>('/api/v1/issues/', issueData);
      
      // Clear cache after creating new issue
      this.clearCache();
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to create issue', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to create issue', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('Issue creation validation failed', error.details?.detail);
      }
      throw error;
    }
  }

  /**
   * Update an issue
   */
  async updateIssue(issueId: string, updates: IssueUpdate): Promise<Issue> {
    try {
      const response = await apiService.put<Issue>(`/api/v1/issues/${issueId}`, updates);
      
      // Clear cache after updating issue
      this.clearCache();
      
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to update issue', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to update issue', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Issue not found', 404, 'NOT_FOUND');
      }
      if (error instanceof ApiError && error.status === 422) {
        throw new ValidationError('Issue update validation failed', error.details?.detail);
      }
      throw error;
    }
  }

  /**
   * Resolve an issue
   */
  async resolveIssue(issueId: string): Promise<Issue> {
    try {
      return await this.updateIssue(issueId, { resolved: true });
    } catch (error) {
      throw error;
    }
  }

  /**
   * Get issue by ID
   */
  async getIssueById(issueId: string): Promise<Issue> {
    try {
      const response = await apiService.get<Issue>(`/api/v1/issues/${issueId}`);
      return response;
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to access issue', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to access issue', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Issue not found', 404, 'NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Delete issue (soft delete)
   */
  async deleteIssue(issueId: string): Promise<void> {
    try {
      await apiService.delete(`/api/v1/issues/${issueId}`);
      
      // Clear cache after deleting issue
      this.clearCache();
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        throw new ApiError('Authentication required to delete issue', 401, 'UNAUTHORIZED');
      }
      if (error instanceof ApiError && error.status === 403) {
        throw new ApiError('Insufficient permissions to delete issue', 403, 'FORBIDDEN');
      }
      if (error instanceof ApiError && error.status === 404) {
        throw new ApiError('Issue not found', 404, 'NOT_FOUND');
      }
      throw error;
    }
  }

  /**
   * Get unresolved issues
   */
  async getUnresolvedIssues(userId: string, params: Omit<IssueQueryParams, 'resolved'> = {}): Promise<Issue[]> {
    return this.getUserIssues(userId, { ...params, resolved: false });
  }

  /**
   * Get critical issues
   */
  async getCriticalIssues(userId: string, params: Omit<IssueQueryParams, 'severity'> = {}): Promise<Issue[]> {
    return this.getUserIssues(userId, { ...params, severity: IssueSeverity.CRITICAL });
  }

  /**
   * Get issues by severity
   */
  async getIssuesBySeverity(userId: string, severity: IssueSeverity, params: Omit<IssueQueryParams, 'severity'> = {}): Promise<Issue[]> {
    return this.getUserIssues(userId, { ...params, severity });
  }

  /**
   * Check if user has unresolved critical issues
   */
  async hasCriticalIssues(userId: string): Promise<boolean> {
    try {
      const criticalIssues = await this.getCriticalIssues(userId, { resolved: false });
      return criticalIssues.length > 0;
    } catch (error) {
      return false;
    }
  }

  /**
   * Generate cache key for query parameters
   */
  private generateCacheKey(userId: string, params: IssueQueryParams): string {
    return `${userId}:${JSON.stringify(params)}`;
  }

  /**
   * Clear issues cache
   */
  clearCache(): void {
    this.issuesCache.clear();
  }

  /**
   * Cache issues for a specific query
   */
  private cacheIssues(key: string, issues: Issue[]): void {
    this.issuesCache.set(key, issues);
  }

  /**
   * Create health issue from metric data
   */
  createHealthIssueFromMetrics(metrics: {
    heartRate?: number;
    oxygenLevel?: number;
    temperature?: number;
    sleepHours?: number;
    timestamp: string;
  }): IssueCreate | null {
    const issues: string[] = [];
    let severity = IssueSeverity.LOW;

    // Check for abnormal heart rate
    if (metrics.heartRate && (metrics.heartRate < 60 || metrics.heartRate > 100)) {
      issues.push(`Heart rate ${metrics.heartRate} BPM`);
      if (metrics.heartRate < 50 || metrics.heartRate > 120) {
        severity = IssueSeverity.CRITICAL;
      } else {
        severity = IssueSeverity.MODERATE;
      }
    }

    // Check for low oxygen
    if (metrics.oxygenLevel && metrics.oxygenLevel < 95) {
      issues.push(`Oxygen level ${metrics.oxygenLevel}%`);
      if (metrics.oxygenLevel < 90) {
        severity = IssueSeverity.CRITICAL;
      } else {
        severity = IssueSeverity.MODERATE;
      }
    }

    // Check for abnormal temperature
    if (metrics.temperature && (metrics.temperature < 36.0 || metrics.temperature > 37.5)) {
      issues.push(`Temperature ${metrics.temperature}°C`);
      if (metrics.temperature < 35.0 || metrics.temperature > 38.5) {
        severity = IssueSeverity.CRITICAL;
      } else {
        severity = IssueSeverity.MODERATE;
      }
    }

    // Check for insufficient sleep
    if (metrics.sleepHours && metrics.sleepHours < 6) {
      issues.push(`Sleep ${metrics.sleepHours} hours`);
      if (metrics.sleepHours < 4) {
        severity = IssueSeverity.MODERATE;
      } else {
        severity = IssueSeverity.LOW;
      }
    }

    if (issues.length > 0) {
      return {
        issue_type: 'health_anomaly',
        description: `Health anomalies detected: ${issues.join(', ')}`,
        severity,
        detected_at: metrics.timestamp,
      };
    }

    return null;
  }

  /**
   * Validate issue data
   */
  validateIssue(issue: IssueCreate): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!issue.issue_type || issue.issue_type.trim() === '') {
      errors.push('Issue type is required');
    }

    if (!issue.description || issue.description.trim() === '') {
      errors.push('Description is required');
    }

    if (!issue.severity) {
      errors.push('Severity is required');
    }

    if (!issue.detected_at) {
      errors.push('Detection timestamp is required');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }
}

// Singleton instance
export const issueService = new IssueService();