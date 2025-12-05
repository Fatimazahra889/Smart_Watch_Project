import React from 'react';
import { View, Text, StyleSheet, ActivityIndicator } from 'react-native';
import { Loader } from 'lucide-react-native';
import colors from '@/constants/colors';

interface LoadingSpinnerProps {
  size?: 'small' | 'large' | number;
  color?: string;
  text?: string;
  overlay?: boolean;
  fullScreen?: boolean;
}

/**
 * Loading Spinner Component
 * Provides consistent loading indicators throughout the app
 */
export const LoadingSpinner: React.FC<LoadingSpinnerProps> = ({
  size = 'large',
  color = colors.primary,
  text,
  overlay = false,
  fullScreen = false,
}) => {
  const spinner = (
    <View style={[
      styles.container,
      overlay && styles.overlay,
      fullScreen && styles.fullScreen,
    ]}>
      <View style={styles.spinnerContainer}>
        <ActivityIndicator 
          size={size} 
          color={color} 
          style={styles.spinner}
        />
        {text && (
          <Text style={styles.text}>{text}</Text>
        )}
      </View>
    </View>
  );

  if (overlay) {
    return (
      <View style={styles.overlayContainer}>
        {spinner}
      </View>
    );
  }

  return spinner;
};

const styles = StyleSheet.create({
  overlayContainer: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    zIndex: 999,
  },
  container: {
    justifyContent: 'center',
    alignItems: 'center',
    padding: 20,
  },
  overlay: {
    backgroundColor: 'rgba(255, 255, 255, 0.9)',
    borderRadius: 12,
    margin: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 8,
    elevation: 4,
  },
  fullScreen: {
    flex: 1,
    backgroundColor: colors.background,
  },
  spinnerContainer: {
    alignItems: 'center',
    gap: 16,
  },
  spinner: {
    marginBottom: 8,
  },
  text: {
    fontSize: 16,
    color: colors.textMuted,
    textAlign: 'center',
  },
});

export default LoadingSpinner;