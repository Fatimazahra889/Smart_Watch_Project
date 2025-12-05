import { HistoricalData } from "@/contexts/HealthDataContext";
import { ChartDataset, ChartDataPoint, ReferenceLine } from "@/types/charts";
import { TimePeriod } from "@/types/analytics";
import colors from "@/constants/colors";

export interface HealthMetricConfig {
  key: string;
  label: string;
  color: string;
  unit: string;
  referenceLines?: ReferenceLine[];
  chartType?: 'line' | 'bar' | 'area';
}

export const HEALTH_METRICS_CONFIG: Record<string, HealthMetricConfig> = {
  heartRate: {
    key: 'heartRate',
    label: 'Heart Rate',
    color: colors.heart,
    unit: 'BPM',
    referenceLines: [
      { value: 60, label: 'Min', color: colors.success, strokeDasharray: '4,4' },
      { value: 100, label: 'Max', color: colors.warning, strokeDasharray: '4,4' },
    ],
    chartType: 'line'
  },
  oxygenLevel: {
    key: 'oxygenLevel',
    label: 'Oxygen Saturation',
    color: colors.success,
    unit: 'SpO₂ %',
    referenceLines: [
      { value: 95, label: 'Healthy Min', color: colors.warning, strokeDasharray: '4,4' },
      { value: 100, label: 'Max', color: colors.success, strokeDasharray: '4,4' },
    ],
    chartType: 'line'
  },
  steps: {
    key: 'steps',
    label: 'Step Count',
    color: colors.primary,
    unit: 'steps',
    chartType: 'bar'
  },
  sleepHours: {
    key: 'sleepHours',
    label: 'Sleep Hours',
    color: colors.secondary,
    unit: 'hours',
    referenceLines: [
      { value: 7, label: 'Target', color: colors.success, strokeDasharray: '4,4' },
      { value: 9, label: 'Max', color: colors.warning, strokeDasharray: '4,4' },
    ],
    chartType: 'bar'
  },
  temperature: {
    key: 'temperature',
    label: 'Body Temperature',
    color: colors.warning,
    unit: '°C',
    referenceLines: [
      { value: 36.5, label: 'Normal', color: colors.success, strokeDasharray: '4,4' },
      { value: 37.5, label: 'Fever', color: colors.danger, strokeDasharray: '4,4' },
    ],
    chartType: 'line'
  }
};

export const transformHealthDataToChartData = (
  historicalData: HistoricalData[],
  metricKey: string,
  timePeriod: TimePeriod = 'daily'
): ChartDataset => {
  const config = HEALTH_METRICS_CONFIG[metricKey];
  if (!config) {
    throw new Error(`Unknown metric key: ${metricKey}`);
  }

  const dataPoints: ChartDataPoint[] = historicalData.map((data, index) => {
    const value = data[metricKey as keyof HistoricalData] as number;
    const timestamp = data.timestamp.getTime();
    
    return {
      x: index,
      y: value,
      timestamp,
      label: formatTimestamp(data.timestamp, timePeriod),
      metadata: {
        unit: config.unit,
        originalValue: value
      }
    };
  });

  return {
    label: config.label,
    data: dataPoints,
    color: config.color,
    type: config.chartType
  };
};

export const transformMultipleMetricsToChartData = (
  historicalData: HistoricalData[],
  metricKeys: string[],
  timePeriod: TimePeriod = 'daily'
): ChartDataset[] => {
  return metricKeys.map(key => transformHealthDataToChartData(historicalData, key, timePeriod));
};

export const getReferenceLinesForMetric = (metricKey: string): ReferenceLine[] => {
  return HEALTH_METRICS_CONFIG[metricKey]?.referenceLines || [];
};

export const getChartTypeForMetric = (metricKey: string): 'line' | 'bar' | 'area' => {
  return HEALTH_METRICS_CONFIG[metricKey]?.chartType || 'line';
};

export const formatTimestamp = (timestamp: Date, timePeriod: TimePeriod): string => {
  switch (timePeriod) {
    case 'daily':
      return timestamp.toLocaleTimeString('en-US', { 
        hour: '2-digit', 
        minute: '2-digit',
        hour12: false 
      });
    case 'weekly':
      return timestamp.toLocaleDateString('en-US', { 
        weekday: 'short',
        month: 'short',
        day: 'numeric'
      });
    case 'monthly':
      return timestamp.toLocaleDateString('en-US', { 
        month: 'short',
        day: 'numeric'
      });
    default:
      return timestamp.toLocaleTimeString('en-US', { 
        hour: '2-digit', 
        minute: '2-digit',
        hour12: false 
      });
  }
};

export const generateMockChartData = (
  metricKey: string,
  timePeriod: TimePeriod = 'daily',
  pointCount: number = 24
): ChartDataset => {
  const config = HEALTH_METRICS_CONFIG[metricKey];
  if (!config) {
    throw new Error(`Unknown metric key: ${metricKey}`);
  }

  const now = new Date();
  const dataPoints: ChartDataPoint[] = [];
  
  // Generate realistic mock data based on metric type
  let baseValue: number;
  let variance: number;
  
  switch (metricKey) {
    case 'heartRate':
      baseValue = 72;
      variance = 15;
      break;
    case 'oxygenLevel':
      baseValue = 98;
      variance = 2;
      break;
    case 'steps':
      baseValue = 5000;
      variance = 3000;
      break;
    case 'sleepHours':
      baseValue = 7.5;
      variance = 2;
      break;
    case 'temperature':
      baseValue = 36.8;
      variance = 0.5;
      break;
    default:
      baseValue = 50;
      variance = 20;
  }

  for (let i = 0; i < pointCount; i++) {
    const timestamp = new Date(now.getTime() - (pointCount - i - 1) * getTimeInterval(timePeriod));
    const randomFactor = Math.sin(i * 0.5) + (Math.random() - 0.5) * 0.5;
    const value = baseValue + randomFactor * variance;
    
    // Ensure values stay within reasonable bounds
    const boundedValue = Math.max(0, Math.min(value, baseValue * 2));
    
    dataPoints.push({
      x: i,
      y: Math.round(boundedValue * 10) / 10,
      timestamp: timestamp.getTime(),
      label: formatTimestamp(timestamp, timePeriod),
      metadata: {
        unit: config.unit,
        isMock: true
      }
    });
  }

  return {
    label: config.label,
    data: dataPoints,
    color: config.color,
    type: config.chartType
  };
};

const getTimeInterval = (timePeriod: TimePeriod): number => {
  switch (timePeriod) {
    case 'daily':
      return 60 * 60 * 1000; // 1 hour
    case 'weekly':
      return 24 * 60 * 60 * 1000; // 1 day
    case 'monthly':
      return 24 * 60 * 60 * 1000; // 1 day
    default:
      return 60 * 60 * 1000; // 1 hour
  }
};

export const getMultiMetricConfig = (metricKeys: string[]) => {
  return metricKeys.map(key => ({
    key,
    label: HEALTH_METRICS_CONFIG[key]?.label || key,
    color: HEALTH_METRICS_CONFIG[key]?.color || colors.primary,
    unit: HEALTH_METRICS_CONFIG[key]?.unit || '',
    visible: true
  }));
};