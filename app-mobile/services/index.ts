// Service exports
export { apiService } from './api';
export { authService } from './auth.service';
export { deviceService } from './device.service';
export { issueService } from './issue.service';
export { metricService } from './metric.service';
export { networkService } from './network.service';
export { offlineStorageService } from './offline-storage.service';
export { queueService } from './queue.service';
export { synchronizationService } from './sync.service';
export { tokenManagerService } from './token-manager.service';
export { tokenService } from './token.service';
export { userService } from './user.service';

// Export types for external use
export type {
  MetricCreate,
  MetricBatchCreate,
  MetricResponse,
  MetricSummary,
  HealthPredictionResponse,
  MetricQueryParams
} from './metric.service';

// Export enums and values
export { MetricType } from './metric.service';

export type {
  Token,
  LoginCredentials,
  EmailVerificationRequest,
  ResendCodeRequest
} from './auth.service';

export type {
  User,
  UserCreate,
  UserUpdate,
  EmailVerificationRequest as UserEmailVerificationRequest,
  ResendCodeRequest as UserResendCodeRequest
} from './user.service';

export type { Device } from './device.service';

export type {
  ApiError,
  NetworkError,
  ValidationError,
  ApiErrorResponse
} from './api';
