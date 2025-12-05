import React from 'react';
import { render } from '@testing-library/react-native';
import { SummaryStatistics } from '../SummaryStatistics';
import { PeriodSummary } from '@/types/analytics';

// Mock data for testing
const mockSummary: PeriodSummary = {
  period: 'daily',
  metrics: {
    heartRate: { avg: 72, min: 65, max: 85, trend: 'stable' },
    steps: { total: 8500, avgDaily: 8500, trend: 'up' },
    sleep: { totalHours: 7.5, avgHours: 7.5, quality: 'good' },
    oxygenLevel: { avg: 98, min: 95, max: 100, trend: 'stable' },
    temperature: { avg: 36.8, min: 36.2, max: 37.2, trend: 'stable' },
    activityKmh: { avg: 3.2, max: 8.5, trend: 'stable' as const },
    idleSeconds: { total: 7200, avgDaily: 7200, trend: 'down' as const },
  },
  healthScore: 85,
  recommendations: [
    'Maintain regular sleep schedule',
    'Increase daily step count by 2000 steps',
    'Stay hydrated throughout the day',
  ],
};

describe('SummaryStatistics', () => {
  it('renders loading state correctly', () => {
    const { getByText } = render(
      <SummaryStatistics isLoading={true} timePeriod="daily" />
    );
    expect(getByText('Loading summary...')).toBeTruthy();
  });

  it('renders error state correctly', () => {
    const { getByText } = render(
      <SummaryStatistics error="Failed to load" timePeriod="daily" />
    );
    expect(getByText('Failed to load summary data')).toBeTruthy();
    expect(getByText('Failed to load')).toBeTruthy();
  });

  it('renders empty state correctly', () => {
    const { getByText } = render(
      <SummaryStatistics timePeriod="daily" />
    );
    expect(getByText('No summary data available')).toBeTruthy();
  });

  it('renders summary data correctly', () => {
    const { getByText } = render(
      <SummaryStatistics summary={mockSummary} timePeriod="daily" />
    );

    // Check health score
    expect(getByText('Health Score')).toBeTruthy();
    expect(getByText('85')).toBeTruthy();
    expect(getByText('/100')).toBeTruthy();
    expect(getByText('Excellent')).toBeTruthy();

    // Check key metrics
    expect(getByText('Heart Rate')).toBeTruthy();
    expect(getByText('72')).toBeTruthy();
    expect(getByText('BPM')).toBeTruthy();

    expect(getByText('Steps')).toBeTruthy();
    expect(getByText('8,500')).toBeTruthy();

    expect(getByText('Sleep')).toBeTruthy();
    expect(getByText('7.5')).toBeTruthy();
    expect(getByText('hrs')).toBeTruthy();

    // Check health status indicators
    expect(getByText('Health Status')).toBeTruthy();
    expect(getByText('Heart Rate')).toBeTruthy();
    expect(getByText('Oxygen Level')).toBeTruthy();
    expect(getByText('Body Temperature')).toBeTruthy();

    // Check recommendations
    expect(getByText('Recommendations')).toBeTruthy();
    expect(getByText('Maintain regular sleep schedule')).toBeTruthy();
    expect(getByText('Increase daily step count by 2000 steps')).toBeTruthy();
    expect(getByText('Stay hydrated throughout the day')).toBeTruthy();
  });

  it('renders different health score descriptions correctly', () => {
    const poorSummary = { ...mockSummary, healthScore: 55 };
    const { getByText } = render(
      <SummaryStatistics summary={poorSummary} timePeriod="daily" />
    );
    expect(getByText('Good')).toBeTruthy();

    const criticalSummary = { ...mockSummary, healthScore: 45 };
    const { getByText: getByTextCritical } = render(
      <SummaryStatistics summary={criticalSummary} timePeriod="daily" />
    );
    expect(getByTextCritical('Needs Improvement')).toBeTruthy();
  });
});