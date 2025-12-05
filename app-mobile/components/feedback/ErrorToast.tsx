import React, { useEffect, useState } from 'react';
import { View, Text, StyleSheet, TouchableOpacity, Animated } from 'react-native';
import { X, AlertTriangle, Info, CheckCircle } from 'lucide-react-native';
import colors from '@/constants/colors';

export type ToastType = 'error' | 'warning' | 'info' | 'success';

interface ErrorToastProps {
  message: string;
  type?: ToastType;
  duration?: number;
  onClose?: () => void;
  show?: boolean;
  action?: {
    label: string;
    onPress: () => void;
  };
}

/**
 * Error Toast Component
 * Displays temporary error/success messages to the user
 */
export const ErrorToast: React.FC<ErrorToastProps> = ({
  message,
  type = 'error',
  duration = 5000,
  onClose,
  show = true,
  action,
}) => {
  const [visible, setVisible] = useState(show);
  const fadeAnim = useState(new Animated.Value(0))[0];
  const slideAnim = useState(new Animated.Value(-100))[0];

  useEffect(() => {
    if (show) {
      setVisible(true);
      // Animate in
      Animated.parallel([
        Animated.timing(fadeAnim, {
          toValue: 1,
          duration: 300,
          useNativeDriver: true,
        }),
        Animated.timing(slideAnim, {
          toValue: 0,
          duration: 300,
          useNativeDriver: true,
        }),
      ]).start();

      // Auto dismiss after duration
      const timer = setTimeout(() => {
        handleClose();
      }, duration);

      return () => clearTimeout(timer);
    } else {
      handleClose();
    }
  }, [show, duration]);

  const handleClose = () => {
    Animated.parallel([
      Animated.timing(fadeAnim, {
        toValue: 0,
        duration: 200,
        useNativeDriver: true,
      }),
      Animated.timing(slideAnim, {
        toValue: -100,
        duration: 200,
        useNativeDriver: true,
      }),
    ]).start(() => {
      setVisible(false);
      onClose?.();
    });
  };

  const getToastStyles = () => {
    switch (type) {
      case 'error':
        return {
          backgroundColor: '#FFEBEE',
          borderColor: colors.danger,
          icon: AlertTriangle,
          iconColor: colors.danger,
        };
      case 'warning':
        return {
          backgroundColor: '#FFF3E0',
          borderColor: colors.warning,
          icon: AlertTriangle,
          iconColor: colors.warning,
        };
      case 'info':
        return {
          backgroundColor: '#E3F2FD',
          borderColor: colors.primary,
          icon: Info,
          iconColor: colors.primary,
        };
      case 'success':
        return {
          backgroundColor: '#E8F5E8',
          borderColor: colors.success,
          icon: CheckCircle,
          iconColor: colors.success,
        };
      default:
        return {
          backgroundColor: '#FFEBEE',
          borderColor: colors.danger,
          icon: AlertTriangle,
          iconColor: colors.danger,
        };
    }
  };

  const toastStyles = getToastStyles();
  const IconComponent = toastStyles.icon;

  if (!visible) return null;

  return (
    <Animated.View
      style={[
        styles.container,
        {
          backgroundColor: toastStyles.backgroundColor,
          borderColor: toastStyles.borderColor,
          opacity: fadeAnim,
          transform: [{ translateY: slideAnim }],
        },
      ]}
    >
      <View style={styles.content}>
        <View style={styles.messageContainer}>
          <IconComponent 
            size={20} 
            color={toastStyles.iconColor} 
            style={styles.icon}
          />
          <Text style={styles.message}>{message}</Text>
        </View>

        <View style={styles.actions}>
          {action && (
            <TouchableOpacity 
              style={styles.actionButton}
              onPress={action.onPress}
            >
              <Text style={[styles.actionText, { color: toastStyles.iconColor }]}>
                {action.label}
              </Text>
            </TouchableOpacity>
          )}
          
          <TouchableOpacity 
            style={styles.closeButton}
            onPress={handleClose}
          >
            <X size={16} color={toastStyles.iconColor} />
          </TouchableOpacity>
        </View>
      </View>
    </Animated.View>
  );
};

const styles = StyleSheet.create({
  container: {
    position: 'absolute',
    top: 60,
    left: 16,
    right: 16,
    borderWidth: 1,
    borderRadius: 12,
    padding: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.15,
    shadowRadius: 12,
    elevation: 8,
    zIndex: 1000,
  },
  content: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    justifyContent: 'space-between',
  },
  messageContainer: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginRight: 12,
  },
  icon: {
    marginRight: 12,
    marginTop: 2,
  },
  message: {
    flex: 1,
    fontSize: 14,
    lineHeight: 20,
    color: colors.text,
    fontWeight: '500',
  },
  actions: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  actionButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 6,
  },
  actionText: {
    fontSize: 14,
    fontWeight: '600',
  },
  closeButton: {
    padding: 4,
    borderRadius: 6,
  },
});

export default ErrorToast;