import colors from "@/constants/colors";
import { SummaryCardProps } from "@/types/analytics";
import { TrendingDown, TrendingUp, Minus } from "lucide-react-native";
import React from "react";
import { StyleSheet, Text, View } from "react-native";
import { LoadingSpinner } from "../feedback/LoadingSpinner";

export const SummaryCard: React.FC<SummaryCardProps> = ({
  title,
  value,
  unit,
  trend,
  comparison,
  icon,
  color,
  isLoading = false,
}) => {
  const getTrendIcon = () => {
    switch (trend) {
      case 'up':
        return <TrendingUp size={16} color={colors.success} />;
      case 'down':
        return <TrendingDown size={16} color={colors.danger} />;
      case 'stable':
        return <Minus size={16} color={colors.textMuted} />;
      default:
        return null;
    }
  };

  const getTrendColor = () => {
    switch (trend) {
      case 'up':
        return colors.success;
      case 'down':
        return colors.danger;
      case 'stable':
        return colors.textMuted;
      default:
        return colors.textMuted;
    }
  };

  if (isLoading) {
    return (
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <View style={[styles.iconContainer, { backgroundColor: `${color}20` }]}>
            {icon}
          </View>
          <LoadingSpinner size="small" />
        </View>
        <View style={styles.valueContainer}>
          <Text style={styles.title}>{title}</Text>
          <View style={styles.loadingValue}>
            <LoadingSpinner size="small" />
          </View>
        </View>
      </View>
    );
  }

  return (
    <View style={styles.card}>
      <View style={styles.cardHeader}>
        <View style={[styles.iconContainer, { backgroundColor: `${color}20` }]}>
          {icon}
        </View>
        {trend && getTrendIcon()}
      </View>
      
      <View style={styles.valueContainer}>
        <Text style={styles.title}>{title}</Text>
        <View style={styles.valueRow}>
          <Text style={styles.value}>{value}</Text>
          {unit && <Text style={styles.unit}>{unit}</Text>}
        </View>
        
        {comparison && (
          <Text style={[styles.comparison, { color: getTrendColor() }]}>
            {comparison}
          </Text>
        )}
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  card: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    marginBottom: 12,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
    minHeight: 120,
  },
  cardHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 12,
  },
  iconContainer: {
    width: 40,
    height: 40,
    borderRadius: 12,
    justifyContent: "center",
    alignItems: "center",
  },
  valueContainer: {
    flex: 1,
  },
  title: {
    fontSize: 14,
    fontWeight: "600" as const,
    color: colors.textMuted,
    marginBottom: 4,
  },
  valueRow: {
    flexDirection: "row",
    alignItems: "baseline",
    marginBottom: 4,
  },
  value: {
    fontSize: 24,
    fontWeight: "700" as const,
    color: colors.text,
    marginRight: 4,
  },
  unit: {
    fontSize: 14,
    color: colors.textMuted,
    fontWeight: "500" as const,
  },
  comparison: {
    fontSize: 12,
    fontWeight: "500" as const,
  },
  loadingValue: {
    height: 32,
    justifyContent: "center",
  },
});