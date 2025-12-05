import colors from "@/constants/colors";
import { useAuth } from "@/contexts/AuthContext";
import { useHealthData } from "@/contexts/HealthDataContext";
import { useMetricsSummary } from "@/hooks/useMetricsSummary";
import { SummaryStatistics } from "@/components/analytics/SummaryStatistics";
import { InteractiveLineChart } from "@/components/analytics/InteractiveLineChart";
import { BarChart } from "@/components/analytics/BarChart";
import { MultiMetricChart } from "@/components/analytics/MultiMetricChart";
import {
  transformHealthDataToChartData,
  transformMultipleMetricsToChartData,
  getReferenceLinesForMetric,
  getMultiMetricConfig,
} from "@/utils/chartDataTransformers";
import { useRouter } from "expo-router";
import React, { useEffect, useMemo, useState } from "react";
import {
  ActivityIndicator,
  Dimensions,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { useSafeAreaInsets } from "react-native-safe-area-context";

const { width } = Dimensions.get("window");
const CHART_WIDTH = width - 40; // 20px padding on each side
const CHART_HEIGHT = 200;

type TimeFilter = "daily" | "weekly" | "monthly";

export default function AnalyticsScreen() {
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const { historicalData } = useHealthData();
  const { isAuthenticated, isLoading } = useAuth();
  const [timeFilter, setTimeFilter] = useState<TimeFilter>("daily");
  const [showSummary, setShowSummary] = useState(true);
  
  const { summary, metricData, isLoading: summaryLoading, error: summaryError } = useMetricsSummary(timeFilter);

  const filteredData = useMemo(() => {
    const now = new Date();
    let hoursBack = 24;

    if (timeFilter === "weekly") hoursBack = 24 * 7;
    if (timeFilter === "monthly") hoursBack = 24 * 30;

    const cutoff = new Date(now.getTime() - hoursBack * 60 * 60 * 1000);
    return historicalData.filter((d) => d.timestamp >= cutoff);
  }, [historicalData, timeFilter]);

  // Transform health data to chart datasets
  const chartData = useMemo(() => {
    if (filteredData.length === 0) return {};

    return {
      heartRate: transformHealthDataToChartData(filteredData, 'heartRate', timeFilter),
      oxygenLevel: transformHealthDataToChartData(filteredData, 'oxygenLevel', timeFilter),
      steps: transformHealthDataToChartData(filteredData, 'steps', timeFilter),
      sleepHours: transformHealthDataToChartData(filteredData, 'sleepHours', timeFilter),
      temperature: transformHealthDataToChartData(filteredData, 'temperature', timeFilter),
    };
  }, [filteredData, timeFilter]);

  // Multi-metric chart data for comparison
  const multiMetricData = useMemo(() => {
    if (filteredData.length === 0) return [];
    
    return transformMultipleMetricsToChartData(
      filteredData,
      ['heartRate', 'oxygenLevel', 'temperature'],
      timeFilter
    );
  }, [filteredData, timeFilter]);

  const multiMetricConfig = useMemo(() => {
    return getMultiMetricConfig(['heartRate', 'oxygenLevel', 'temperature']);
  }, []);

  const chartConfig = {
    width: CHART_WIDTH,
    height: CHART_HEIGHT,
    padding: { top: 20, right: 20, bottom: 40, left: 40 },
    showGrid: true,
    showAxes: true,
    showTooltip: true,
    animate: true,
    animationDuration: 500,
  };

  const handleDataPointPress = (dataPoint: any, datasetIndex: number) => {
    console.log('Data point pressed:', dataPoint, 'Dataset:', datasetIndex);
  };

  const handleMetricToggle = (metricKey: string, visible: boolean) => {
    console.log('Metric toggled:', metricKey, 'Visible:', visible);
  };

  // Redirect to signin if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      router.replace('/signin');
    }
  }, [isAuthenticated, isLoading, router]);

  // Show loading while checking authentication
  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <ActivityIndicator size="large" color={colors.primary} />
        <Text style={styles.loadingText}>Loading...</Text>
      </View>
    );
  }

  // Don't render content if not authenticated (will redirect)
  if (!isAuthenticated) {
    return null;
  }


  return (
    <View style={styles.container}>
      <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
        <Text style={styles.headerTitle}>Analytics</Text>
        {/* <View style={styles.headerActions}>
          <TouchableOpacity
            style={styles.viewToggle}
            onPress={() => setShowSummary(!showSummary)}
          >
            <Text style={styles.viewToggleText}>
              {showSummary ? 'Show Charts' : 'Show Summary'}
            </Text>
          </TouchableOpacity>
          <TouchableOpacity style={styles.exportButton}>
            <Download size={20} color={colors.primary} />
            <Text style={styles.exportText}>Export</Text>
          </TouchableOpacity>
        </View> */}
      </View>

      <View style={styles.filterContainer}>
        <TouchableOpacity
          style={[
            styles.filterButton,
            timeFilter === "daily" && styles.filterButtonActive,
          ]}
          onPress={() => setTimeFilter("daily")}
        >
          <Text
            style={[
              styles.filterText,
              timeFilter === "daily" && styles.filterTextActive,
            ]}
          >
            Daily
          </Text>
        </TouchableOpacity>
        <TouchableOpacity
          style={[
            styles.filterButton,
            timeFilter === "weekly" && styles.filterButtonActive,
          ]}
          onPress={() => setTimeFilter("weekly")}
        >
          <Text
            style={[
              styles.filterText,
              timeFilter === "weekly" && styles.filterTextActive,
            ]}
          >
            Weekly
          </Text>
        </TouchableOpacity>
        <TouchableOpacity
          style={[
            styles.filterButton,
            timeFilter === "monthly" && styles.filterButtonActive,
          ]}
          onPress={() => setTimeFilter("monthly")}
        >
          <Text
            style={[
              styles.filterText,
              timeFilter === "monthly" && styles.filterTextActive,
            ]}
          >
            Monthly
          </Text>
        </TouchableOpacity>
      </View>

      {showSummary ? (
        <SummaryStatistics
          summary={summary || undefined}
          metricData={metricData}
          isLoading={summaryLoading}
          error={summaryError || undefined}
          timePeriod={timeFilter}
        />
      ) : (
        <ScrollView
          contentContainerStyle={styles.contentContainer}
          showsVerticalScrollIndicator={false}
        >

        {/* Heart Rate Chart */}
        {chartData.heartRate && (
          <InteractiveLineChart
            data={[chartData.heartRate]}
            config={chartConfig}
            referenceLines={getReferenceLinesForMetric('heartRate')}
            timePeriod={timeFilter}
            onDataPointPress={handleDataPointPress}
            showPoints={true}
            smooth={true}
            strokeWidth={3}
          />
        )}

        {/* Oxygen Level Chart */}
        {chartData.oxygenLevel && (
          <InteractiveLineChart
            data={[chartData.oxygenLevel]}
            config={chartConfig}
            referenceLines={getReferenceLinesForMetric('oxygenLevel')}
            timePeriod={timeFilter}
            onDataPointPress={handleDataPointPress}
            showPoints={true}
            smooth={true}
            strokeWidth={3}
          />
        )}

        {/* Temperature Chart */}
        {chartData.temperature && (
          <InteractiveLineChart
            data={[chartData.temperature]}
            config={chartConfig}
            referenceLines={getReferenceLinesForMetric('temperature')}
            timePeriod={timeFilter}
            onDataPointPress={handleDataPointPress}
            showPoints={true}
            smooth={true}
            strokeWidth={3}
          />
        )}

        {/* Multi-Metric Comparison Chart */}
        {multiMetricData.length > 0 && (
          <MultiMetricChart
            data={multiMetricData}
            config={chartConfig}
            metrics={multiMetricConfig}
            timePeriod={timeFilter}
            onDataPointPress={handleDataPointPress}
            onMetricToggle={handleMetricToggle}
          />
        )}
        </ScrollView>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  contentContainer: {
    paddingBottom: 32,
  },
  headerActions: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },
  viewToggle: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    backgroundColor: colors.white,
    borderRadius: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  viewToggleText: {
    fontSize: 12,
    fontWeight: "600" as const,
    color: colors.primary,
  },
  loadingContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: colors.background,
  },
  loadingText: {
    fontSize: 16,
    color: colors.textMuted,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingBottom: 20,
  },
  headerTitle: {
    fontSize: 28,
    fontWeight: "700" as const,
    color: colors.text,
  },
  exportButton: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    paddingHorizontal: 16,
    paddingVertical: 8,
    backgroundColor: colors.white,
    borderRadius: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  exportText: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: colors.primary,
  },
  filterContainer: {
    flexDirection: "row",
    marginHorizontal: 20,
    marginBottom: 24,
    backgroundColor: colors.white,
    borderRadius: 12,
    padding: 4,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  filterButton: {
    flex: 1,
    paddingVertical: 10,
    alignItems: "center",
    borderRadius: 8,
  },
  filterButtonActive: {
    backgroundColor: colors.primary,
  },
  filterText: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: colors.textMuted,
  },
  filterTextActive: {
    color: colors.white,
  },
  chartCard: {
    marginHorizontal: 20,
    marginBottom: 24,
    padding: 16,
    backgroundColor: colors.white,
    borderRadius: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
    overflow: 'hidden', // Prevent chart overflow
  },
  chartHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 16,
  },
  chartTitle: {
    fontSize: 16,
    fontWeight: "700" as const,
    color: colors.text,
  },
  trendBadge: {
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
    paddingHorizontal: 8,
    paddingVertical: 4,
    backgroundColor: colors.secondary,
    borderRadius: 12,
  },
  trendText: {
    fontSize: 12,
    fontWeight: "600" as const,
    color: colors.success,
  },
  chartUnitContainer: {
    flexDirection: "row",
    justifyContent: "center",
    alignItems: "baseline",
    marginTop: 8,
  },
  chartUnit: {
    fontSize: 12,
    color: colors.textMuted,
    marginLeft: 4,
  },
});
