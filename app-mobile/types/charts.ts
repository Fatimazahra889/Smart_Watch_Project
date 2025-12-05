import { TimePeriod } from "./analytics";

export type ChartType = 'line' | 'bar' | 'area' | 'multi';

export interface ChartDataPoint {
  x: number | string;
  y: number;
  timestamp?: number;
  label?: string;
  metadata?: Record<string, any>;
}

export interface ChartDataset {
  label: string;
  data: ChartDataPoint[];
  color: string;
  type?: ChartType;
  visible?: boolean;
  strokeWidth?: number;
  fillOpacity?: number;
}

export interface ChartConfig {
  width: number;
  height: number;
  padding?: {
    top: number;
    right: number;
    bottom: number;
    left: number;
  };
  showGrid?: boolean;
  showAxes?: boolean;
  showTooltip?: boolean;
  animate?: boolean;
  animationDuration?: number;
}

export interface ReferenceLine {
  value: number;
  label: string;
  color: string;
  strokeWidth?: number;
  strokeDasharray?: string;
}

export interface ChartTooltip {
  x: number;
  y: number;
  visible: boolean;
  dataPoints: Array<{
    datasetLabel: string;
    value: number;
    color: string;
    unit?: string;
  }>;
}

export interface ZoomState {
  scale: number;
  translateX: number;
  translateY: number;
  minScale: number;
  maxScale: number;
}

export interface ChartInteractionState {
  isPanning: boolean;
  isZooming: boolean;
  lastTouchX: number;
  lastTouchY: number;
  zoomState: ZoomState;
  selectedDataPoint?: ChartDataPoint;
}

export interface BaseChartProps {
  data: ChartDataset[];
  config: ChartConfig;
  referenceLines?: ReferenceLine[];
  timePeriod?: TimePeriod;
  onDataPointPress?: (dataPoint: ChartDataPoint, datasetIndex: number) => void;
  onZoomChange?: (zoomState: ZoomState) => void;
  isLoading?: boolean;
  error?: string;
}

export interface LineChartProps extends BaseChartProps {
  showPoints?: boolean;
  smooth?: boolean;
  strokeWidth?: number;
}

export interface BarChartProps extends BaseChartProps {
  barWidth?: number;
  barSpacing?: number;
  showValues?: boolean;
}

export interface AreaChartProps extends BaseChartProps {
  gradient?: boolean;
  fillOpacity?: number;
  strokeWidth?: number;
}

export interface MultiMetricChartProps extends BaseChartProps {
  metrics: Array<{
    key: string;
    label: string;
    color: string;
    unit: string;
    visible: boolean;
  }>;
  onMetricToggle?: (metricKey: string, visible: boolean) => void;
}

export interface ChartAggregationOptions {
  method: 'average' | 'sum' | 'max' | 'min' | 'latest';
  interval: 'hour' | 'day' | 'week' | 'month';
  maxPoints?: number;
}

export interface ChartDataAggregator {
  aggregate(data: ChartDataPoint[], options: ChartAggregationOptions): ChartDataPoint[];
  sample(data: ChartDataPoint[], maxPoints: number): ChartDataPoint[];
}

export interface ChartAnimationConfig {
  duration: number;
  easing?: 'linear' | 'easeIn' | 'easeOut' | 'easeInOut';
  delay?: number;
}

export interface ChartPerformanceConfig {
  maxDataPoints: number;
  enableVirtualization: boolean;
  debounceRender: number;
  enableDataSampling: boolean;
}