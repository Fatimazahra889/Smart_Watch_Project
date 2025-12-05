import colors from "@/constants/colors";
import { HealthStatusIndicatorProps } from "@/types/analytics";
import { TrendingDown, TrendingUp, Minus, AlertTriangle, CheckCircle, XCircle } from "lucide-react-native";
import React from "react";
import { StyleSheet, Text, View } from "react-native";

export const HealthStatusIndicator: React.FC<HealthStatusIndicatorProps> = ({
  metric,
  value,
  unit,
  status,
  trend,
}) => {
  const getStatusIcon = () => {
    switch (status) {
      case 'good':
        return <CheckCircle size={16} color={colors.success} />;
      case 'warning':
        return <AlertTriangle size={16} color={colors.warning} />;
      case 'danger':
        return <XCircle size={16} color={colors.danger} />;
      default:
        return <CheckCircle size={16} color={colors.success} />;
    }
  };

  const getStatusColor = () => {
    switch (status) {
      case 'good':
        return colors.success;
      case 'warning':
        return colors.warning;
      case 'danger':
        return colors.danger;
      default:
        return colors.success;
    }
  };

  const getStatusText = () => {
    switch (status) {
      case 'good':
        return 'Good';
      case 'warning':
        return 'Warning';
      case 'danger':
        return 'Critical';
      default:
        return 'Good';
    }
  };

  const getTrendIcon = () => {
    switch (trend) {
      case 'up':
        return <TrendingUp size={14} color={colors.success} />;
      case 'down':
        return <TrendingDown size={14} color={colors.danger} />;
      case 'stable':
        return <Minus size={14} color={colors.textMuted} />;
      default:
        return null;
    }
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.metricName}>{metric}</Text>
        {getTrendIcon()}
      </View>
      
      <View style={styles.valueRow}>
        <Text style={styles.value}>
          {value}
          <Text style={styles.unit}>{unit}</Text>
        </Text>
        <View style={[styles.statusBadge, { backgroundColor: `${getStatusColor()}20` }]}>
          {getStatusIcon()}
          <Text style={[styles.statusText, { color: getStatusColor() }]}>
            {getStatusText()}
          </Text>
        </View>
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    backgroundColor: colors.white,
    borderRadius: 12,
    padding: 16,
    marginBottom: 8,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.05,
    shadowRadius: 4,
    elevation: 2,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 8,
  },
  metricName: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: colors.text,
  },
  valueRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
  },
  value: {
    fontSize: 18,
    fontWeight: "700" as const,
    color: colors.text,
  },
  unit: {
    fontSize: 14,
    color: colors.textMuted,
    fontWeight: "500" as const,
    marginLeft: 2,
  },
  statusBadge: {
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 8,
  },
  statusText: {
    fontSize: 12,
    fontWeight: "600" as const,
  },
});