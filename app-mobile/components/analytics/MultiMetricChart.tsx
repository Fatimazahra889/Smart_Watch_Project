import colors from "@/constants/colors";
import { MultiMetricChartProps, ChartTooltip } from "@/types/charts";
import { ChartDataAggregator, calculateChartScales, generatePathData, debounce, getOptimalAggregation } from "@/utils/chartUtils";
import React, { useState, useRef, useCallback, useMemo } from "react";
import {
  Dimensions,
  StyleSheet,
  View,
  Text,
  TouchableOpacity,
  PanResponder,
  GestureResponderEvent,
} from "react-native";
import Svg, { 
  Path, 
  Line, 
  Circle,
  Rect,
  Text as SvgText,
  G,
  Defs,
  LinearGradient,
  Stop
} from "react-native-svg";
import { LoadingSpinner } from "../feedback/LoadingSpinner";
import { Check, X } from "lucide-react-native";

const { width: SCREEN_WIDTH } = Dimensions.get("window");
const CHART_WIDTH = SCREEN_WIDTH - 40;
const CHART_HEIGHT = 200;

export const MultiMetricChart: React.FC<MultiMetricChartProps> = ({
  data,
  config,
  metrics,
  referenceLines = [],
  timePeriod = "daily",
  onDataPointPress,
  onZoomChange,
  onMetricToggle,
  isLoading = false,
  error,
}) => {
  const [tooltip, setTooltip] = useState<ChartTooltip>({
    x: 0,
    y: 0,
    visible: false,
    dataPoints: [],
  });

  const [visibleMetrics, setVisibleMetrics] = useState<Record<string, boolean>>(
    metrics.reduce((acc, metric) => ({ ...acc, [metric.key]: metric.visible }), {})
  );

  const svgRef = useRef<any>(null);

  // Filter data based on visible metrics
  const filteredData = useMemo(() => {
    return data.filter((dataset, index) => {
      const metricKey = metrics[index]?.key;
      return visibleMetrics[metricKey] !== false;
    });
  }, [data, metrics, visibleMetrics]);

  // Process data with aggregation
  const processedData = useMemo(() => {
    if (filteredData.length === 0) return [];
    
    return filteredData.map(dataset => {
      const aggregationOptions = getOptimalAggregation(
        dataset.data.length,
        timePeriod,
        100
      );
      
      const aggregatedData = ChartDataAggregator.aggregate(
        dataset.data,
        aggregationOptions
      );
      
      return {
        ...dataset,
        data: aggregatedData,
      };
    });
  }, [filteredData, timePeriod]);

  // Calculate scales and domains for all visible data
  const { xScale, yScale, xDomain, yDomain } = useMemo(() => {
    const allDataPoints = processedData.flatMap(dataset => dataset.data);
    return calculateChartScales(
      allDataPoints,
      CHART_WIDTH,
      CHART_HEIGHT,
      config.padding || { top: 20, right: 20, bottom: 40, left: 40 }
    );
  }, [processedData, config.padding]);

  // Generate path data for each dataset
  const pathData = useMemo(() => {
    return processedData.map(dataset => {
      const points = dataset.data.map((point, index) => ({
        x: xScale(index),
        y: yScale(point.y),
      }));
      return generatePathData(points, true); // Smooth lines for multi-metric charts
    });
  }, [processedData, xScale, yScale]);

  // Handle metric toggle
  const handleMetricToggle = useCallback((metricKey: string) => {
    const newVisibleMetrics = {
      ...visibleMetrics,
      [metricKey]: !visibleMetrics[metricKey],
    };
    setVisibleMetrics(newVisibleMetrics);
    
    if (onMetricToggle) {
      onMetricToggle(metricKey, !visibleMetrics[metricKey]);
    }
  }, [visibleMetrics, onMetricToggle]);

  // Disabled touch events for tooltips and interactions
  const handleTouch = useCallback(() => {
    // No-op function to disable touch interactions
  }, []);

  // Disabled pan responder
  const panResponder = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => false,
      onMoveShouldSetPanResponder: () => false,
    })
  ).current;

  // Render grid lines
  const renderGrid = () => {
    const gridLines = [];
    const xStep = CHART_WIDTH / 6;
    const yStep = CHART_HEIGHT / 4;

    // Horizontal grid lines
    for (let i = 1; i < 4; i++) {
      gridLines.push(
        <Line
          key={`h-grid-${i}`}
          x1={0}
          y1={i * yStep}
          x2={CHART_WIDTH}
          y2={i * yStep}
          stroke={colors.border}
          strokeWidth="1"
          strokeDasharray="2,2"
        />
      );
    }

    // Vertical grid lines
    for (let i = 1; i < 6; i++) {
      gridLines.push(
        <Line
          key={`v-grid-${i}`}
          x1={i * xStep}
          y1={0}
          x2={i * xStep}
          y2={CHART_HEIGHT}
          stroke={colors.border}
          strokeWidth="1"
          strokeDasharray="2,2"
        />
      );
    }

    return gridLines;
  };

  // Render reference lines
  const renderReferenceLines = () => {
    return referenceLines.map((line, index) => {
      const y = yScale(line.value);
      return (
        <G key={`ref-line-${index}`}>
          <Line
            x1={0}
            y1={y}
            x2={CHART_WIDTH}
            y2={y}
            stroke={line.color}
            strokeWidth={line.strokeWidth || 2}
            strokeDasharray={line.strokeDasharray || "4,4"}
          />
          <SvgText
            x={CHART_WIDTH - 40}
            y={y - 8}
            fontSize="10"
            fill={line.color}
            fontWeight="600"
          >
            {line.label}
          </SvgText>
        </G>
      );
    });
  };

  // Render tooltip
  const renderTooltip = () => {
    if (!tooltip.visible || tooltip.dataPoints.length === 0) return null;

    return (
      <G>
        {/* Tooltip line */}
        <Line
          x1={tooltip.x}
          y1={0}
          x2={tooltip.x}
          y2={CHART_HEIGHT}
          stroke={colors.textMuted}
          strokeWidth="1"
          strokeDasharray="2,2"
        />
        
        {/* Tooltip background */}
        <Rect
          x={tooltip.x - 80}
          y={tooltip.y - 100}
          width={160}
          height={tooltip.dataPoints.length * 15 + 20}
          fill={colors.white}
          stroke={colors.border}
          strokeWidth="1"
          rx="8"
        />
        
        {/* Tooltip content */}
        {tooltip.dataPoints.map((dataPoint, index) => (
          <SvgText
            key={index}
            x={tooltip.x - 70}
            y={tooltip.y - 80 + index * 15}
            fontSize="10"
            fill={colors.text}
          >
            <SvgText fill={dataPoint.color}>{dataPoint.datasetLabel}: </SvgText>
            <SvgText fontWeight="600">
              {dataPoint.value.toFixed(1)}
              {dataPoint.unit}
            </SvgText>
          </SvgText>
        ))}
      </G>
    );
  };

  // Render metric toggles
  const renderMetricToggles = () => {
    return (
      <View style={styles.metricToggles}>
        <Text style={styles.metricTogglesTitle}>Metrics:</Text>
        <View style={styles.toggleContainer}>
          {metrics.map((metric, index) => (
            <TouchableOpacity
              key={metric.key}
              style={[
                styles.toggleButton,
                visibleMetrics[metric.key] && styles.toggleButtonActive
              ]}
              onPress={() => handleMetricToggle(metric.key)}
            >
              <View style={styles.toggleContent}>
                <View 
                  style={[
                    styles.toggleColor, 
                    { backgroundColor: metric.color }
                  ]} 
                />
                <Text style={[
                  styles.toggleText,
                  visibleMetrics[metric.key] && styles.toggleTextActive
                ]}>
                  {metric.label}
                </Text>
                {visibleMetrics[metric.key] ? (
                  <Check size={14} color={colors.success} />
                ) : (
                  <X size={14} color={colors.textMuted} />
                )}
              </View>
            </TouchableOpacity>
          ))}
        </View>
      </View>
    );
  };

  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <LoadingSpinner size="small" />
        <Text style={styles.loadingText}>Loading chart...</Text>
      </View>
    );
  }

  if (error) {
    return (
      <View style={styles.errorContainer}>
        <Text style={styles.errorText}>Failed to load chart data</Text>
        <Text style={styles.errorSubtext}>{error}</Text>
      </View>
    );
  }

  if (processedData.length === 0) {
    return (
      <View style={styles.emptyContainer}>
        <Text style={styles.emptyText}>No data available</Text>
        {renderMetricToggles()}
      </View>
    );
  }

  return (
    <View style={styles.container}>
      {renderMetricToggles()}
      
      <View
        {...panResponder.panHandlers}
        style={styles.chartContainer}
        onTouchStart={handleTouch}
        onTouchMove={handleTouch}
      >
        <Svg
          ref={svgRef}
          width={CHART_WIDTH}
          height={CHART_HEIGHT}
          style={styles.svg}
        >
          <Defs>
            {processedData.map((dataset, index) => (
              <LinearGradient
                key={`gradient-${index}`}
                id={`gradient-${index}`}
                x1="0%"
                y1="0%"
                x2="0%"
                y2="100%"
              >
                <Stop offset="0%" stopColor={dataset.color} stopOpacity="0.3" />
                <Stop offset="100%" stopColor={dataset.color} stopOpacity="0" />
              </LinearGradient>
            ))}
          </Defs>

          {/* Grid */}
          {config.showGrid !== false && renderGrid()}

          {/* Reference lines */}
          {renderReferenceLines()}

          {/* Chart paths */}
          {pathData.map((path, index) => (
            <G key={`path-${index}`}>
              <Path
                d={path}
                stroke={processedData[index].color}
                strokeWidth="3"
                fill="none"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              
              {/* Data points */}
              {processedData[index].data.map((point, pointIndex) => (
                <Circle
                  key={`point-${index}-${pointIndex}`}
                  cx={xScale(pointIndex)}
                  cy={yScale(point.y)}
                  r="3"
                  fill={processedData[index].color}
                  stroke={colors.white}
                  strokeWidth="1"
                />
              ))}
            </G>
          ))}

          {/* Tooltip */}
          {renderTooltip()}

          {/* Axes */}
          {config.showAxes !== false && (
            <G>
              <Line
                x1={0}
                y1={CHART_HEIGHT}
                x2={CHART_WIDTH}
                y2={CHART_HEIGHT}
                stroke={colors.border}
                strokeWidth="2"
              />
              <Line
                x1={0}
                y1={0}
                x2={0}
                y2={CHART_HEIGHT}
                stroke={colors.border}
                strokeWidth="2"
              />
            </G>
          )}
        </Svg>
      </View>

      {/* X-axis labels */}
      <View style={styles.xAxisLabels}>
        {processedData[0].data
          .filter((_, index) => index % Math.ceil(processedData[0].data.length / 5) === 0)
          .map((point, index) => (
            <Text key={index} style={styles.xAxisLabel}>
              {point.label || point.x.toString()}
            </Text>
          ))}
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    marginBottom: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  chartContainer: {
    width: CHART_WIDTH,
    height: CHART_HEIGHT,
  },
  svg: {
    backgroundColor: "transparent",
  },
  loadingContainer: {
    height: CHART_HEIGHT,
    justifyContent: "center",
    alignItems: "center",
  },
  loadingText: {
    marginTop: 8,
    fontSize: 14,
    color: colors.textMuted,
  },
  errorContainer: {
    height: CHART_HEIGHT,
    justifyContent: "center",
    alignItems: "center",
    padding: 20,
  },
  errorText: {
    fontSize: 16,
    color: colors.danger,
    fontWeight: "600",
    marginBottom: 4,
  },
  errorSubtext: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: "center",
  },
  emptyContainer: {
    height: CHART_HEIGHT,
    justifyContent: "center",
    alignItems: "center",
  },
  emptyText: {
    fontSize: 16,
    color: colors.textMuted,
  },
  xAxisLabels: {
    flexDirection: "row",
    justifyContent: "space-between",
    marginTop: 8,
    paddingHorizontal: 10,
  },
  xAxisLabel: {
    fontSize: 10,
    color: colors.textMuted,
    fontWeight: "500",
  },
  metricToggles: {
    marginBottom: 16,
  },
  metricTogglesTitle: {
    fontSize: 14,
    fontWeight: "600",
    color: colors.text,
    marginBottom: 8,
  },
  toggleContainer: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 8,
  },
  toggleButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    backgroundColor: colors.background,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: colors.border,
  },
  toggleButtonActive: {
    backgroundColor: colors.primary + "20",
    borderColor: colors.primary,
  },
  toggleContent: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
  },
  toggleColor: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  toggleText: {
    fontSize: 12,
    color: colors.textMuted,
    fontWeight: "500",
  },
  toggleTextActive: {
    color: colors.primary,
    fontWeight: "600",
  },
});