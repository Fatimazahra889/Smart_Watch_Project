import colors from "@/constants/colors";
import { PeriodSummary, TimePeriod, DailyMetricData } from "@/types/analytics";
import { MetricType, metricService, HealthPredictionResponse } from "@/services/metric.service";
import { MetricSummaryData } from "@/hooks/useMetricsSummary";
import {
  Heart,
  Footprints,
  Moon,
  Activity,
  Thermometer,
  Zap,
  Timer,
  TrendingUp,
  TrendingDown,
  Minus,
  RefreshCw,
  AlertTriangle,
  CheckCircle,
  Info
} from "lucide-react-native";
import React, { useState, useCallback } from "react";
import { StyleSheet, Text, View, ScrollView, Dimensions, TouchableOpacity, Alert } from "react-native";
import { SummaryCard } from "./SummaryCard";
import { HealthStatusIndicator } from "./HealthStatusIndicator";
import { LoadingSpinner } from "../feedback/LoadingSpinner";
import { InteractiveLineChart } from "./InteractiveLineChart";

interface SummaryStatisticsProps {
  summary?: PeriodSummary;
  metricData?: Record<MetricType, MetricSummaryData | null>;
  isLoading?: boolean;
  error?: string;
  timePeriod: TimePeriod;
}

export const SummaryStatistics: React.FC<SummaryStatisticsProps> = ({
  summary,
  metricData,
  isLoading = false,
  error,
  timePeriod,
}) => {
  const [healthPrediction, setHealthPrediction] = useState<HealthPredictionResponse | null>(null);
  const [isPredicting, setIsPredicting] = useState(false);
  const [predictionError, setPredictionError] = useState<string | null>(null);
  const getHealthStatus = (metric: string, value: number): 'good' | 'warning' | 'danger' => {
    switch (metric) {
      case 'heartRate':
        if (value < 60 || value > 100) return 'warning';
        if (value < 50 || value > 120) return 'danger';
        return 'good';
      case 'oxygenLevel':
        if (value < 95) return 'warning';
        if (value < 90) return 'danger';
        return 'good';
      case 'temperature':
        if (value < 36 || value > 37.5) return 'warning';
        if (value < 35 || value > 38.5) return 'danger';
        return 'good';
      case 'steps':
        if (value < 5000) return 'warning';
        if (value < 3000) return 'danger';
        return 'good';
      case 'sleep':
        if (value < 6 || value > 9) return 'warning';
        if (value < 4 || value > 10) return 'danger';
        return 'good';
      default:
        return 'good';
    }
  };

  const getComparisonText = (trend: 'up' | 'down' | 'stable', metric: string): string => {
    const periodText = timePeriod === 'daily' ? 'yesterday' :
                      timePeriod === 'weekly' ? 'last week' : 'last month';
    
    switch (trend) {
      case 'up':
        return `↑ vs ${periodText}`;
      case 'down':
        return `↓ vs ${periodText}`;
      case 'stable':
        return `→ vs ${periodText}`;
      default:
        return '';
    }
  };

  const formatDateLabel = (dateString: string, period: TimePeriod): string => {
    const date = new Date(dateString);
    
    switch (period) {
      case 'daily':
        return date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
      case 'weekly':
        return `Week ${Math.ceil(date.getDate() / 7)}`;
      case 'monthly':
        return date.toLocaleDateString('en-US', { month: 'short' });
      default:
        return date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
    }
  };

  const getMetricColor = (metricType: MetricType): string => {
    switch (metricType) {
      case MetricType.HEART_RATE:
        return colors.heart;
      case MetricType.SPO2:
        return colors.primary;
      case MetricType.SKIN_TEMPERATURE:
        return colors.warning;
      case MetricType.STEPS:
        return colors.success;
      case MetricType.SLEEP:
        return colors.secondary;
      default:
        return colors.primary;
    }
  };

  const getMetricDisplayName = (metricType: MetricType): string => {
    switch (metricType) {
      case MetricType.HEART_RATE:
        return 'Heart Rate';
      case MetricType.SPO2:
        return 'Oxygen Level';
      case MetricType.SKIN_TEMPERATURE:
        return 'Temperature';
      case MetricType.STEPS:
        return 'Steps';
      case MetricType.SLEEP:
        return 'Sleep';
      default:
        return metricType;
    }
  };

  const renderMetricChart = (metricType: MetricType) => {
    const metricSummary = metricData?.[metricType];
    
    if (!metricSummary || metricSummary.daily_summaries.length === 0) {
      return (
        <View style={styles.chartContainer}>
          <Text style={styles.noDataText}>No data available for {getMetricDisplayName(metricType)}</Text>
        </View>
      );
    }

    const chartData = [{
      label: getMetricDisplayName(metricType),
      color: getMetricColor(metricType),
      data: metricSummary.daily_summaries.map((day: DailyMetricData, index: number) => ({
        x: index,
        y: day.average_value,
        label: formatDateLabel(day.date, timePeriod),
        metadata: { unit: day.unit }
      }))
    }];

    // Calculate dynamic width based on container padding
    const chartWidth = Dimensions.get('window').width - 72; // 20px padding * 2 + 16px container padding * 2

    return (
      <View style={styles.chartContainer}>
        <Text style={styles.chartTitle}>{getMetricDisplayName(metricType)} Trend</Text>
        <InteractiveLineChart
          data={chartData}
          config={{
            width: chartWidth,
            height: 200,
            showGrid: true,
            showAxes: true,
            padding: { top: 20, right: 20, bottom: 40, left: 40 }
          }}
          timePeriod={timePeriod}
          showPoints={true}
          smooth={true}
        />
      </View>
    );
  };

  // Helper functions for health prediction display
  const formatRiskLevel = (riskLevel: string): { text: string; color: string } => {
    switch (riskLevel) {
      case 'low':
        return { text: 'Low Risk', color: colors.success };
      case 'medium':
        return { text: 'Medium Risk', color: colors.warning };
      case 'high':
        return { text: 'High Risk', color: colors.danger };
      default:
        return { text: 'Unknown', color: colors.textMuted };
    }
  };

  const formatPredictionResult = (result: number): { text: string; color: string } => {
    switch (result) {
      case 0:
        return { text: 'Normal', color: colors.success };
      case 1:
        return { text: 'Sick', color: colors.warning };
      case 2:
        return { text: 'Life-Threatening', color: colors.danger };
      default:
        return { text: 'Unknown', color: colors.textMuted };
    }
  };

  const formatMetricStatus = (isHealthy: boolean): { text: string; color: string } => {
    return isHealthy
      ? { text: 'Normal', color: colors.success }
      : { text: 'Warning', color: colors.warning };
  };

  // Handle health prediction request
  const handleHealthPrediction = useCallback(async () => {
    setIsPredicting(true);
    setPredictionError(null);
    
    try {
      const prediction = await metricService.getHealthPrediction({
        include_metrics: true,
        prediction_horizon_hours: 24
      });
      setHealthPrediction(prediction);
    } catch (error) {
      console.error('Health prediction failed:', error);
      setPredictionError(
        error instanceof Error ? error.message : 'Failed to get health prediction'
      );
      Alert.alert(
        'Health Prediction Error',
        'Unable to generate health prediction. Please try again later.',
        [{ text: 'OK' }]
      );
    } finally {
      setIsPredicting(false);
    }
  }, []);

  if (error) {
    return (
      <View style={styles.errorContainer}>
        <Text style={styles.errorText}>Failed to load summary data</Text>
        <Text style={styles.errorSubtext}>{error}</Text>
      </View>
    );
  }

  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <LoadingSpinner size="large" />
        <Text style={styles.loadingText}>Loading summary...</Text>
      </View>
    );
  }

  if (!summary) {
    return (
      <View style={styles.emptyContainer}>
        <Text style={styles.emptyText}>No summary data available</Text>
      </View>
    );
  }

  const { metrics, healthScore, recommendations } = summary;

  return (
    <ScrollView style={styles.container} showsVerticalScrollIndicator={false}>
      {/* Health Score */}
      {/* Daily Trends Charts */}
      <View style={styles.chartsSection}>
        <Text style={styles.sectionTitle}>Daily Trends</Text>
        {renderMetricChart(MetricType.HEART_RATE)}
        {renderMetricChart(MetricType.SPO2)}
        {renderMetricChart(MetricType.SKIN_TEMPERATURE)}
        {renderMetricChart(MetricType.STEPS)}
        {renderMetricChart(MetricType.SLEEP)}
      </View>
      
      {/* <View style={styles.cardsGrid}>
        <SummaryCard
          title="Heart Rate"
          value={Math.round(metrics.heartRate.avg)}
          unit="BPM"
          trend={metrics.heartRate.trend}
          comparison={getComparisonText(metrics.heartRate.trend, 'heartRate')}
          icon={<Heart size={20} color={colors.heart} />}
          color={colors.heart}
        />
        
        <SummaryCard
          title="Steps"
          value={metrics.steps.total.toLocaleString()}
          trend={metrics.steps.trend}
          comparison={getComparisonText(metrics.steps.trend, 'steps')}
          icon={<Footprints size={20} color={colors.success} />}
          color={colors.success}
        />
        
        <SummaryCard
          title="Sleep"
          value={metrics.sleep.avgHours.toFixed(1)}
          unit="hrs"
          trend={metrics.sleep.quality === 'good' ? 'up' : metrics.sleep.quality === 'poor' ? 'down' : 'stable'}
          comparison={`${metrics.sleep.quality} quality`}
          icon={<Moon size={20} color={colors.secondary} />}
          color={colors.secondary}
        />
        
        <SummaryCard
          title="Oxygen"
          value={Math.round(metrics.oxygenLevel.avg)}
          unit="%"
          trend={metrics.oxygenLevel.trend}
          comparison={getComparisonText(metrics.oxygenLevel.trend, 'oxygenLevel')}
          icon={<Activity size={20} color={colors.primary} />}
          color={colors.primary}
        />
        
        <SummaryCard
          title="Temperature"
          value={metrics.temperature.avg.toFixed(1)}
          unit="°C"
          trend={metrics.temperature.trend}
          comparison={getComparisonText(metrics.temperature.trend, 'temperature')}
          icon={<Thermometer size={20} color={colors.warning} />}
          color={colors.warning}
        />
       
      </View>

      <View style={styles.healthStatusSection}>
        <Text style={styles.sectionTitle}>Health Status</Text>
        <HealthStatusIndicator
          metric="Heart Rate"
          value={Math.round(metrics.heartRate.avg)}
          unit="BPM"
          status={getHealthStatus('heartRate', metrics.heartRate.avg)}
          trend={metrics.heartRate.trend}
        />
        <HealthStatusIndicator
          metric="Oxygen Level"
          value={Math.round(metrics.oxygenLevel.avg)}
          unit="%"
          status={getHealthStatus('oxygenLevel', metrics.oxygenLevel.avg)}
          trend={metrics.oxygenLevel.trend}
        />
        <HealthStatusIndicator
          metric="Body Temperature"
          value={metrics.temperature.avg.toFixed(1)}
          unit="°C"
          status={getHealthStatus('temperature', metrics.temperature.avg)}
          trend={metrics.temperature.trend}
        />
        <HealthStatusIndicator
          metric="Daily Steps"
          value={metrics.steps.avgDaily.toLocaleString()}
          unit="steps"
          status={getHealthStatus('steps', metrics.steps.avgDaily)}
          trend={metrics.steps.trend}
        />
        <HealthStatusIndicator
          metric="Sleep Hours"
          value={metrics.sleep.avgHours.toFixed(1)}
          unit="hrs"
          status={getHealthStatus('sleep', metrics.sleep.avgHours)}
          trend={metrics.sleep.quality === 'good' ? 'up' : metrics.sleep.quality === 'poor' ? 'down' : 'stable'}
        />
      </View> */}
    </ScrollView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingHorizontal: 20,
    paddingTop: 16,
  },
  loadingContainer: {
    padding: 40,
    alignItems: 'center',
    justifyContent: 'center',
  },
  loadingText: {
    marginTop: 12,
    fontSize: 16,
    color: colors.textMuted,
  },
  errorContainer: {
    padding: 40,
    alignItems: 'center',
    justifyContent: 'center',
  },
  errorText: {
    fontSize: 16,
    color: colors.danger,
    fontWeight: '600' as const,
    marginBottom: 8,
  },
  errorSubtext: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: 'center',
  },
  emptyContainer: {
    padding: 40,
    alignItems: 'center',
    justifyContent: 'center',
  },
  emptyText: {
    fontSize: 16,
    color: colors.textMuted,
  },
  healthScoreCard: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 20,
    marginBottom: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  healthScoreHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  healthScoreTitle: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
  },
  scoreContainer: {
    flexDirection: 'row',
    alignItems: 'baseline',
  },
  healthScore: {
    fontSize: 28,
    fontWeight: '700' as const,
    color: colors.text,
  },
  healthScoreUnit: {
    fontSize: 16,
    color: colors.textMuted,
    marginLeft: 4,
  },
  scoreBar: {
    height: 8,
    backgroundColor: colors.border,
    borderRadius: 4,
    overflow: 'hidden',
    marginBottom: 8,
  },
  scoreFill: {
    height: '100%',
    borderRadius: 4,
  },
  healthScoreDescription: {
    fontSize: 14,
    color: colors.textMuted,
    fontWeight: '500' as const,
  },
  cardsGrid: {
    marginBottom: 20,
  },
  healthStatusSection: {
    marginBottom: 20,
  },
  chartsSection: {
    marginBottom: 20,
  },
  chartContainer: {
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
  chartTitle: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 12,
  },
  noDataText: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: 'center',
    padding: 20,
  },
  recommendationsSection: {
    marginBottom: 20,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '700' as const,
    color: colors.text,
    marginBottom: 16,
  },
  recommendationItem: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginBottom: 8,
    paddingHorizontal: 8,
  },
  recommendationBullet: {
    fontSize: 16,
    color: colors.primary,
    marginRight: 8,
    marginTop: 2,
  },
  recommendationText: {
    flex: 1,
    fontSize: 14,
    color: colors.text,
    lineHeight: 20,
  },
  healthPredictionTrigger: {
    marginTop: 12,
    paddingTop: 12,
    borderTopWidth: 1,
    borderTopColor: colors.border,
  },
  predictingContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
  },
  predictingText: {
    fontSize: 14,
    color: colors.primary,
    marginLeft: 8,
    fontWeight: '500' as const,
  },
  predictContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
  },
  predictText: {
    fontSize: 14,
    color: colors.primary,
    marginRight: 8,
    fontWeight: '500' as const,
  },
  predictionCard: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 20,
    marginBottom: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  predictionHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 16,
  },
  predictionTitle: {
    fontSize: 18,
    fontWeight: '700' as const,
    color: colors.text,
  },
  closeButton: {
    fontSize: 18,
    color: colors.textMuted,
    fontWeight: '600' as const,
  },
  predictionSection: {
    marginBottom: 20,
  },
  predictionSectionTitle: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 12,
  },
  assessmentRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 16,
  },
  assessmentItem: {
    flex: 1,
  },
  assessmentLabel: {
    fontSize: 14,
    color: colors.textMuted,
    marginBottom: 4,
  },
  assessmentValue: {
    fontSize: 18,
    fontWeight: '700' as const,
  },
  riskBadge: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 12,
    alignSelf: 'flex-start',
  },
  riskText: {
    fontSize: 12,
    fontWeight: '600' as const,
  },
  predictionResult: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  predictionResultLabel: {
    fontSize: 14,
    color: colors.text,
    fontWeight: '500' as const,
  },
  confidenceText: {
    fontSize: 16,
    color: colors.primary,
    fontWeight: '700' as const,
  },
  metricAnalysisItem: {
    backgroundColor: colors.background,
    borderRadius: 8,
    padding: 12,
    marginBottom: 8,
  },
  metricAnalysisHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 8,
  },
  metricName: {
    fontSize: 14,
    fontWeight: '600' as const,
    color: colors.text,
    flex: 1,
  },
  statusBadge: {
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 8,
  },
  statusText: {
    fontSize: 10,
    fontWeight: '600' as const,
  },
  metricDetails: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  metricValue: {
    fontSize: 12,
    color: colors.text,
    fontWeight: '500' as const,
  },
  metricScore: {
    fontSize: 12,
    color: colors.textMuted,
  },
  riskFactorItem: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginBottom: 8,
  },
  riskFactorText: {
    flex: 1,
    fontSize: 14,
    color: colors.text,
    marginLeft: 8,
    lineHeight: 20,
  },
  predictionFooter: {
    borderTopWidth: 1,
    borderTopColor: colors.border,
    paddingTop: 12,
    marginTop: 8,
  },
  predictionHorizon: {
    fontSize: 12,
    color: colors.textMuted,
    textAlign: 'center',
    marginBottom: 4,
  },
  modelVersion: {
    fontSize: 10,
    color: colors.textMuted,
    textAlign: 'center',
  },
  errorCard: {
    backgroundColor: colors.danger + '10',
    borderRadius: 12,
    padding: 16,
    marginBottom: 20,
    flexDirection: 'row',
    alignItems: 'center',
    borderWidth: 1,
    borderColor: colors.danger + '20',
  },
  errorCardText: {
    flex: 1,
    fontSize: 14,
    color: colors.danger,
    marginLeft: 12,
    marginRight: 12,
  },
});