import { ChartDataPoint, ChartAggregationOptions } from "@/types/charts";

export class ChartDataAggregator {
  static aggregate(data: ChartDataPoint[], options: ChartAggregationOptions): ChartDataPoint[] {
    if (data.length === 0) return [];

    const { method, interval, maxPoints } = options;
    
    // If we have fewer points than maxPoints, return as is
    if (maxPoints && data.length <= maxPoints) {
      return data;
    }

    // Group data by time intervals
    const grouped = this.groupByInterval(data, interval);
    
    // Apply aggregation method to each group
    const aggregated: ChartDataPoint[] = [];
    
    for (const [intervalKey, points] of Object.entries(grouped)) {
      if (points.length === 0) continue;
      
      let aggregatedValue: number;
      
      switch (method) {
        case 'average':
          aggregatedValue = points.reduce((sum, p) => sum + p.y, 0) / points.length;
          break;
        case 'sum':
          aggregatedValue = points.reduce((sum, p) => sum + p.y, 0);
          break;
        case 'max':
          aggregatedValue = Math.max(...points.map(p => p.y));
          break;
        case 'min':
          aggregatedValue = Math.min(...points.map(p => p.y));
          break;
        case 'latest':
          aggregatedValue = points[points.length - 1].y;
          break;
        default:
          aggregatedValue = points.reduce((sum, p) => sum + p.y, 0) / points.length;
      }
      
      aggregated.push({
        x: intervalKey,
        y: aggregatedValue,
        timestamp: points[0].timestamp,
        label: this.formatIntervalLabel(intervalKey, interval),
        metadata: {
          originalPoints: points.length,
          aggregationMethod: method
        }
      });
    }
    
    return aggregated;
  }

  static sample(data: ChartDataPoint[], maxPoints: number): ChartDataPoint[] {
    if (data.length <= maxPoints) return data;
    
    const step = Math.ceil(data.length / maxPoints);
    const sampled: ChartDataPoint[] = [];
    
    for (let i = 0; i < data.length; i += step) {
      sampled.push(data[i]);
    }
    
    // Ensure we include the last point
    if (sampled[sampled.length - 1] !== data[data.length - 1]) {
      sampled.push(data[data.length - 1]);
    }
    
    return sampled;
  }

  private static groupByInterval(data: ChartDataPoint[], interval: string): Record<string, ChartDataPoint[]> {
    const grouped: Record<string, ChartDataPoint[]> = {};
    
    for (const point of data) {
      const timestamp = point.timestamp || Date.now();
      const date = new Date(timestamp);
      let intervalKey: string;
      
      switch (interval) {
        case 'hour':
          intervalKey = `${date.getFullYear()}-${date.getMonth() + 1}-${date.getDate()} ${date.getHours()}:00`;
          break;
        case 'day':
          intervalKey = `${date.getFullYear()}-${date.getMonth() + 1}-${date.getDate()}`;
          break;
        case 'week':
          const weekStart = new Date(date);
          weekStart.setDate(date.getDate() - date.getDay());
          intervalKey = `${weekStart.getFullYear()}-${weekStart.getMonth() + 1}-${weekStart.getDate()}`;
          break;
        case 'month':
          intervalKey = `${date.getFullYear()}-${date.getMonth() + 1}`;
          break;
        default:
          intervalKey = `${date.getFullYear()}-${date.getMonth() + 1}-${date.getDate()} ${date.getHours()}:00`;
      }
      
      if (!grouped[intervalKey]) {
        grouped[intervalKey] = [];
      }
      grouped[intervalKey].push(point);
    }
    
    return grouped;
  }

  private static formatIntervalLabel(intervalKey: string, interval: string): string {
    switch (interval) {
      case 'hour':
        return intervalKey.split(' ')[1];
      case 'day':
        const [year, month, day] = intervalKey.split('-');
        return `${month}/${day}`;
      case 'week':
        const [weekYear, weekMonth, weekDay] = intervalKey.split('-');
        return `Week of ${weekMonth}/${weekDay}`;
      case 'month':
        const [monthYear, monthNum] = intervalKey.split('-');
        const monthNames = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
        return `${monthNames[parseInt(monthNum) - 1]} ${monthYear}`;
      default:
        return intervalKey;
    }
  }
}

export const calculateChartScales = (
  data: ChartDataPoint[], 
  width: number, 
  height: number,
  padding: { top: number; right: number; bottom: number; left: number }
) => {
  if (data.length === 0) {
    return {
      xScale: (x: number) => x,
      yScale: (y: number) => y,
      xDomain: [0, 1],
      yDomain: [0, 1]
    };
  }

  const xValues = data.map((d, i) => i);
  const yValues = data.map(d => d.y);

  const xMin = Math.min(...xValues);
  const xMax = Math.max(...xValues);
  const yMin = Math.min(...yValues);
  const yMax = Math.max(...yValues);

  const xRange = xMax - xMin || 1;
  const yRange = yMax - yMin || 1;

  const xScale = (x: number) => 
    padding.left + ((x - xMin) / xRange) * (width - padding.left - padding.right);
  
  const yScale = (y: number) => 
    height - padding.bottom - ((y - yMin) / yRange) * (height - padding.top - padding.bottom);

  return {
    xScale,
    yScale,
    xDomain: [xMin, xMax],
    yDomain: [yMin, yMax]
  };
};

export const generatePathData = (
  points: { x: number; y: number }[],
  smooth: boolean = false
): string => {
  if (points.length === 0) return '';

  if (smooth && points.length > 2) {
    return generateSmoothPath(points);
  }

  return points
    .map((point, i) => `${i === 0 ? 'M' : 'L'} ${point.x} ${point.y}`)
    .join(' ');
};

const generateSmoothPath = (points: { x: number; y: number }[]): string => {
  let path = `M ${points[0].x} ${points[0].y}`;
  
  for (let i = 1; i < points.length - 1; i++) {
    const xc = (points[i].x + points[i + 1].x) / 2;
    const yc = (points[i].y + points[i + 1].y) / 2;
    path += ` Q ${points[i].x} ${points[i].y} ${xc} ${yc}`;
  }
  
  if (points.length > 1) {
    path += ` L ${points[points.length - 1].x} ${points[points.length - 1].y}`;
  }
  
  return path;
};

export const debounce = <T extends (...args: any[]) => void>(
  func: T,
  delay: number
): ((...args: Parameters<T>) => void) => {
  let timeoutId: number;
  return (...args: Parameters<T>) => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => func(...args), delay) as unknown as number;
  };
};

export const formatValue = (value: number, unit?: string): string => {
  if (value >= 1000) {
    return `${(value / 1000).toFixed(1)}k${unit ? ` ${unit}` : ''}`;
  }
  return `${value}${unit ? ` ${unit}` : ''}`;
};

export const getOptimalAggregation = (
  dataLength: number,
  timePeriod: string,
  maxPoints: number = 100
): ChartAggregationOptions => {
  if (dataLength <= maxPoints) {
    return {
      method: 'latest',
      interval: 'hour',
      maxPoints
    };
  }

  switch (timePeriod) {
    case 'daily':
      return {
        method: 'average',
        interval: 'hour',
        maxPoints: 24
      };
    case 'weekly':
      return {
        method: 'average',
        interval: 'day',
        maxPoints: 7
      };
    case 'monthly':
      return {
        method: 'average',
        interval: 'day',
        maxPoints: 30
      };
    default:
      return {
        method: 'average',
        interval: 'hour',
        maxPoints
      };
  }
};