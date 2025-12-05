/**
 * Test Runner for Offline Storage Services
 * Executes tests in a React Native compatible way
 */

import { runAllIntegrationTests } from './offline-storage-integration.test';
import { testOfflineStorage } from './offline-storage.test';

/**
 * Run all offline storage tests
 */
async function runAllTests(): Promise<void> {
  console.log('🚀 Starting Offline Storage Test Suite\n');
  console.log('='.repeat(50));
  
  try {
    // Run basic offline storage test
    console.log('\n📋 Running Basic Offline Storage Test...');
    await testOfflineStorage();
    
    // Run comprehensive integration tests
    console.log('\n📋 Running Comprehensive Integration Tests...');
    await runAllIntegrationTests();
    
    console.log('\n' + '='.repeat(50));
    console.log('🎉 ALL TESTS COMPLETED SUCCESSFULLY!');
    console.log('\n✅ Offline Storage Solution is working correctly');
    
  } catch (error) {
    console.error('\n💥 TEST SUITE FAILED:', error);
    throw error;
  }
}

// Export for manual execution
export { runAllTests };

// Run tests if this file is executed directly
if (require.main === module) {
  runAllTests().catch(error => {
    console.error('Test runner failed:', error);
    process.exit(1);
  });
}