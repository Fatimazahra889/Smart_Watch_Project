export type TimePeriod = 'daily' | 'weekly' | 'monthly';
export type TrendDirection = 'up' | 'down' | 'stable';
export type SleepQuality = 'good' | 'fair' | 'poor';

export interface DailyMetricData {
  date: string;
  average_value: number;
  count: number;
  metric_type: string;
  unit: string;
}

export interface MetricSummaryResponse {
  daily_summaries: DailyMetricData[];
  metric_type: string;
  start_date: string;
  end_date: string;
  total_days: number;
  days_with_data: number;
}

export interface PeriodSummary {
  period: TimePeriod;
  metrics: {
    heartRate: { avg: number; min: number; max: number; trend: TrendDirection };
    steps: { total: number; avgDaily: number; trend: TrendDirection };
    sleep: { totalHours: number; avgHours: number; quality: SleepQuality };
    oxygenLevel: { avg: number; min: number; max: number; trend: TrendDirection };
    temperature: { avg: number; min: number; max: number; trend: TrendDirection };
    activityKmh: { avg: number; max: number; trend: TrendDirection };
    idleSeconds: { total: number; avgDaily: number; trend: TrendDirection };
  };
  healthScore: number;
  recommendations: string[];
}

export interface SummaryCardProps {
  title: string;
  value: string | number;
  unit?: string;
  trend?: TrendDirection;
  comparison?: string;
  icon: React.ReactNode;
  color: string;
  isLoading?: boolean;
}

export interface HealthStatusIndicatorProps {
  metric: string;
  value: string | number;
  unit: string;
  status: 'good' | 'warning' | 'danger';
  trend?: TrendDirection;
}

export interface ChartDataPoint {
  date: string;
  value: number;
  label: string;
}

export interface MetricChartData {
  metricType: string;
  data: ChartDataPoint[];
  unit: string;
  color: string;
}