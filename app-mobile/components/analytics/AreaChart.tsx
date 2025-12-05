import colors from "@/constants/colors";
import { AreaChartProps, ChartTooltip } from "@/types/charts";
import { ChartDataAggregator, calculateChartScales, generatePathData, debounce, getOptimalAggregation } from "@/utils/chartUtils";
import React, { useState, useRef, useCallback, useMemo } from "react";
import {
  Dimensions,
  StyleSheet,
  View,
  Text,
  PanResponder,
  GestureResponderEvent,
} from "react-native";
import Svg, {
  Path,
  Line,
  Rect,
  Text as SvgText,
  G,
  Defs,
  LinearGradient,
  Stop
} from "react-native-svg";
import { LoadingSpinner } from "../feedback/LoadingSpinner";

const { width: SCREEN_WIDTH } = Dimensions.get("window");
const CHART_WIDTH = SCREEN_WIDTH - 40;
const CHART_HEIGHT = 200;

export const AreaChart: React.FC<AreaChartProps> = ({
  data,
  config,
  referenceLines = [],
  timePeriod = "daily",
  onDataPointPress,
  onZoomChange,
  isLoading = false,
  error,
  gradient = true,
  fillOpacity = 0.3,
  strokeWidth = 2,
}) => {
  const [tooltip, setTooltip] = useState<ChartTooltip>({
    x: 0,
    y: 0,
    visible: false,
    dataPoints: [],
  });

  const svgRef = useRef<any>(null);

  // Process data with aggregation
  const processedData = useMemo(() => {
    if (data.length === 0) return [];
    
    return data.map(dataset => {
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
  }, [data, timePeriod]);

  // Calculate scales and domains
  const { xScale, yScale, xDomain, yDomain } = useMemo(() => {
    const allDataPoints = processedData.flatMap(dataset => dataset.data);
    return calculateChartScales(
      allDataPoints,
      CHART_WIDTH,
      CHART_HEIGHT,
      config.padding || { top: 20, right: 20, bottom: 40, left: 40 }
    );
  }, [processedData, config.padding]);

  // Generate area path data
  const areaPathData = useMemo(() => {
    return processedData.map(dataset => {
      const points = dataset.data.map((point, index) => ({
        x: xScale(index),
        y: yScale(point.y),
      }));

      if (points.length === 0) return '';

      // Create area path (closed path from line to baseline)
      let path = `M ${points[0].x} ${points[0].y}`;
      
      for (let i = 1; i < points.length; i++) {
        path += ` L ${points[i].x} ${points[i].y}`;
      }
      
      // Close the path to create area
      path += ` L ${points[points.length - 1].x} ${yScale(0)}`;
      path += ` L ${points[0].x} ${yScale(0)}`;
      path += ' Z';

      return path;
    });
  }, [processedData, xScale, yScale]);

  // Generate line path data
  const linePathData = useMemo(() => {
    return processedData.map(dataset => {
      const points = dataset.data.map((point, index) => ({
        x: xScale(index),
        y: yScale(point.y),
      }));
      return generatePathData(points, true); // Smooth lines for area charts
    });
  }, [processedData, xScale, yScale]);

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
          x={tooltip.x - 60}
          y={tooltip.y - 80}
          width={120}
          height={60}
          fill={colors.white}
          stroke={colors.border}
          strokeWidth="1"
          rx="8"
        />
        
        {/* Tooltip content */}
        {tooltip.dataPoints.map((dataPoint, index) => (
          <SvgText
            key={index}
            x={tooltip.x - 50}
            y={tooltip.y - 60 + index * 15}
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
      </View>
    );
  }

  return (
    <View style={styles.container}>
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
                <Stop 
                  offset="0%" 
                  stopColor={dataset.color} 
                  stopOpacity={fillOpacity} 
                />
                <Stop 
                  offset="100%" 
                  stopColor={dataset.color} 
                  stopOpacity="0" 
                />
              </LinearGradient>
            ))}
          </Defs>

          {/* Grid */}
          {config.showGrid !== false && renderGrid()}

          {/* Reference lines */}
          {renderReferenceLines()}

          {/* Area fills */}
          {areaPathData.map((path, index) => (
            <Path
              key={`area-${index}`}
              d={path}
              fill={gradient ? `url(#gradient-${index})` : processedData[index].color}
              fillOpacity={gradient ? 1 : fillOpacity}
            />
          ))}

          {/* Line paths */}
          {linePathData.map((path, index) => (
            <Path
              key={`line-${index}`}
              d={path}
              stroke={processedData[index].color}
              strokeWidth={strokeWidth}
              fill="none"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
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

      {/* Legend */}
      {processedData.length > 1 && (
        <View style={styles.legend}>
          {processedData.map((dataset, index) => (
            <View key={index} style={styles.legendItem}>
              <View 
                style={[
                  styles.legendColor, 
                  { backgroundColor: dataset.color }
                ]} 
              />
              <Text style={styles.legendText}>{dataset.label}</Text>
            </View>
          ))}
        </View>
      )}
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
  legend: {
    flexDirection: "row",
    justifyContent: "center",
    flexWrap: "wrap",
    marginTop: 12,
    gap: 12,
  },
  legendItem: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
  },
  legendColor: {
    width: 12,
    height: 12,
    borderRadius: 2,
  },
  legendText: {
    fontSize: 12,
    color: colors.textMuted,
    fontWeight: "500",
  },
});