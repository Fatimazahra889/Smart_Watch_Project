// Jest setup file for React Native testing
/* eslint-disable @typescript-eslint/no-var-requires */
require('@testing-library/jest-native/extend-expect');

// Mock AsyncStorage
jest.mock('@react-native-async-storage/async-storage', () => ({
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
}));

// Mock React Native modules
jest.mock('react-native/Libraries/Animated/NativeAnimatedHelper');

// Mock fetch for API testing
global.fetch = jest.fn();

// Mock console methods to reduce noise in tests
global.console = {
  ...console,
  // Uncomment to debug specific tests
  // log: jest.fn(),
  // error: jest.fn(),
  // warn: jest.fn(),
};

// Reset all mocks between tests
beforeEach(() => {
  jest.clearAllMocks();
  
  // Reset fetch mock
  fetch.mockReset();
  
  // Reset AsyncStorage mocks
  const AsyncStorage = require('@react-native-async-storage/async-storage');
  AsyncStorage.getItem.mockReset();
  AsyncStorage.setItem.mockReset();
  AsyncStorage.removeItem.mockReset();
  AsyncStorage.clear.mockReset();
});