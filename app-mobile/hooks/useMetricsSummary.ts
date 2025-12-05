import { PeriodSummary, TimePeriod, MetricSummaryResponse } from "@/types/analytics";
import { metricService, MetricType } from "@/services/metric.service";
import { useAuth } from "@/contexts/AuthContext";
import { useState, useEffect, useCallback } from "react";

export interface DailyMetricData {
  date: string;
  average_value: number;
  count: number;
  metric_type: MetricType;
  unit: string;
}

export interface MetricSummaryData {
  daily_summaries: DailyMetricData[];
  metric_type: MetricType;
  start_date: string;
  end_date: string;
  total_days: number;
  days_with_data: number;
}

export const useMetricsSummary = (timePeriod: TimePeriod) => {
  const [summary, setSummary] = useState<PeriodSummary | null>(null);
  const [metricData, setMetricData] = useState<Record<MetricType, MetricSummaryData | null>>({
    [MetricType.SPO2]: null,
    [MetricType.HEART_RATE]: null,
    [MetricType.SKIN_TEMPERATURE]: null,
    [MetricType.STEPS]: null,
    [MetricType.SLEEP]: null,
    [MetricType.AMBIENT_TEMPERATURE]: null,
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const { isAuthenticated } = useAuth();

  const calculateTrend = (current: number, previous: number): 'up' | 'down' | 'stable' => {
    const threshold = 0.05; // 5% change threshold
    const change = (current - previous) / previous;
    
    if (Math.abs(change) < threshold) return 'stable';
    return change > 0 ? 'up' : 'down';
  };

  const getSleepQuality = (avgHours: number): 'good' | 'fair' | 'poor' => {
    if (avgHours >= 7 && avgHours <= 9) return 'good';
    if (avgHours >= 6 && avgHours < 7) return 'fair';
    return 'poor';
  };

  const getDateRange = (period: TimePeriod): { startDate: string; endDate: string } => {
    const endDate = new Date();
    const startDate = new Date();
    
    switch (period) {
      case 'daily':
        startDate.setDate(startDate.getDate() - 7); // Last 7 days
        break;
      case 'weekly':
        startDate.setDate(startDate.getDate() - 28); // Last 4 weeks
        break;
      case 'monthly':
        startDate.setMonth(startDate.getMonth() - 6); // Last 6 months
        break;
    }
    
    return {
      startDate: startDate.toISOString(),
      endDate: endDate.toISOString()
    };
  };

  const fetchMetricSummary = async (metricType: MetricType, dateRange: { startDate: string; endDate: string }): Promise<MetricSummaryData> => {
    try {
      const response = await metricService.getMetricsSummary({
        metric_type: metricType,
        start_date: dateRange.startDate,
        end_date: dateRange.endDate
      });
      return response as MetricSummaryData;
    } catch (error) {
      console.error(`Error fetching ${metricType} summary:`, error);
      // Return empty data structure on error
      return {
        daily_summaries: [],
        metric_type: metricType,
        start_date: dateRange.startDate,
        end_date: dateRange.endDate,
        total_days: 0,
        days_with_data: 0
      };
    }
  };

  const fetchAllMetricsSummary = useCallback(async () => {
    if (!isAuthenticated) return;
    
    setIsLoading(true);
    setError(null);
    
    try {
      const dateRange = getDateRange(timePeriod);
      
      // Fetch all metric types in parallel
      const metricTypes = [
        MetricType.SPO2,
        MetricType.HEART_RATE,
        MetricType.SKIN_TEMPERATURE,
        MetricType.STEPS,
        MetricType.SLEEP
      ];
      
      const promises = metricTypes.map(metricType =>
        fetchMetricSummary(metricType, dateRange)
      );
      
      const results = await Promise.all(promises);
      
      // Update metric data state
      const newMetricData: Record<MetricType, MetricSummaryData | null> = {
        [MetricType.SPO2]: null,
        [MetricType.HEART_RATE]: null,
        [MetricType.SKIN_TEMPERATURE]: null,
        [MetricType.STEPS]: null,
        [MetricType.SLEEP]: null,
        [MetricType.AMBIENT_TEMPERATURE]: null,
      };
      
      results.forEach(result => {
        newMetricData[result.metric_type] = result;
      });
      
      setMetricData(newMetricData);
      
      // Calculate aggregated summary for backward compatibility
      const heartRateData = newMetricData[MetricType.HEART_RATE];
      const stepsData = newMetricData[MetricType.STEPS];
      const sleepData = newMetricData[MetricType.SLEEP];
      const spo2Data = newMetricData[MetricType.SPO2];
      const temperatureData = newMetricData[MetricType.SKIN_TEMPERATURE];
      
      // Calculate averages from daily summaries
      const calculateAverage = (data: MetricSummaryData | null): number => {
        if (!data || data.daily_summaries.length === 0) return 0;
        const sum = data.daily_summaries.reduce((acc, day) => acc + day.average_value, 0);
        return sum / data.daily_summaries.length;
      };
      
      const heartRateAvg = calculateAverage(heartRateData);
      const stepsAvg = calculateAverage(stepsData);
      const sleepAvg = calculateAverage(sleepData);
      const spo2Avg = calculateAverage(spo2Data);
      const temperatureAvg = calculateAverage(temperatureData);
      
      const mockSummary: PeriodSummary = {
        period: timePeriod,
        metrics: {
          heartRate: {
            avg: heartRateAvg || 72,
            min: Math.min(...(heartRateData?.daily_summaries.map(d => d.average_value) || [65])),
            max: Math.max(...(heartRateData?.daily_summaries.map(d => d.average_value) || [85])),
            trend: calculateTrend(heartRateAvg || 72, 70)
          },
          steps: {
            total: stepsAvg ? stepsAvg * (timePeriod === 'daily' ? 7 : timePeriod === 'weekly' ? 28 : 180) : 8500,
            avgDaily: stepsAvg || 8500,
            trend: calculateTrend(stepsAvg || 8500, 8000)
          },
          sleep: {
            totalHours: sleepAvg ? sleepAvg * (timePeriod === 'daily' ? 7 : timePeriod === 'weekly' ? 28 : 180) : 7.5,
            avgHours: sleepAvg || 7.5,
            quality: getSleepQuality(sleepAvg || 7.5)
          },
          oxygenLevel: {
            avg: spo2Avg || 98,
            min: Math.min(...(spo2Data?.daily_summaries.map(d => d.average_value) || [95])),
            max: Math.max(...(spo2Data?.daily_summaries.map(d => d.average_value) || [100])),
            trend: calculateTrend(spo2Avg || 98, 97)
          },
          temperature: {
            avg: temperatureAvg || 36.8,
            min: Math.min(...(temperatureData?.daily_summaries.map(d => d.average_value) || [36.2])),
            max: Math.max(...(temperatureData?.daily_summaries.map(d => d.average_value) || [37.2])),
            trend: calculateTrend(temperatureAvg || 36.8, 36.7)
          },
          activityKmh: {
            avg: 3.2,
            max: 8.5,
            trend: 'stable' as const
          },
          idleSeconds: {
            total: timePeriod === 'daily' ? 7200 : timePeriod === 'weekly' ? 50400 : 216000,
            avgDaily: 7200,
            trend: 'down' as const
          }
        },
        healthScore: 85, // Default health score
        recommendations: [
          'Maintain regular sleep schedule',
          'Increase daily step count by 2000 steps',
          'Stay hydrated throughout the day'
        ]
      };
      
      setSummary(mockSummary);
    } catch (err) {
      console.error('Error fetching metrics summary:', err);
      setError(err instanceof Error ? err.message : 'Failed to load summary data');
    } finally {
      setIsLoading(false);
    }
  }, [timePeriod, isAuthenticated]);

  useEffect(() => {
    fetchAllMetricsSummary();
  }, [fetchAllMetricsSummary]);

  const refetch = useCallback(() => {
    fetchAllMetricsSummary();
  }, [fetchAllMetricsSummary]);

  return {
    summary,
    metricData,
    isLoading,
    error,
    refetch
  };
};