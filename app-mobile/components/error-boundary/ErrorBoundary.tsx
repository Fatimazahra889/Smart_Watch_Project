import React from 'react';
import { View, Text, StyleSheet, TouchableOpacity, ScrollView } from 'react-native';
import { AlertTriangle, RefreshCw, Home } from 'lucide-react-native';
import colors from '@/constants/colors';

interface Props {
  children: React.ReactNode;
  fallback?: React.ComponentType<{ error: Error; resetError: () => void }>;
}

interface State {
  hasError: boolean;
  error: Error | null;
}

/**
 * Global Error Boundary Component
 * Catches JavaScript errors anywhere in the component tree and displays a fallback UI
 */
export class ErrorBoundary extends React.Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
    };
  }

  static getDerivedStateFromError(error: Error): State {
    // Update state so the next render will show the fallback UI
    return {
      hasError: true,
      error,
    };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    // Log error to error reporting service
    console.error('Error Boundary caught an error:', error, errorInfo);
    
    // You can also log to an external service here
    // errorReportingService.logError(error, errorInfo);
  }

  resetError = () => {
    this.setState({
      hasError: false,
      error: null,
    });
  };

  render() {
    if (this.state.hasError) {
      // You can render any custom fallback UI
      if (this.props.fallback) {
        const FallbackComponent = this.props.fallback;
        return <FallbackComponent error={this.state.error!} resetError={this.resetError} />;
      }

      return (
        <View style={styles.container}>
          <ScrollView contentContainerStyle={styles.scrollContent}>
            <View style={styles.errorContainer}>
              <View style={styles.iconContainer}>
                <AlertTriangle size={64} color={colors.danger} />
              </View>
              
              <Text style={styles.title}>Something went wrong</Text>
              
              <Text style={styles.message}>
                We encountered an unexpected error. This might be a temporary issue.
              </Text>

              {this.state.error && (
                <View style={styles.errorDetails}>
                  <Text style={styles.errorMessage}>
                    {this.state.error.message || 'Unknown error occurred'}
                  </Text>
                </View>
              )}

              <View style={styles.actions}>
                <TouchableOpacity 
                  style={[styles.button, styles.primaryButton]}
                  onPress={this.resetError}
                >
                  <RefreshCw size={20} color={colors.white} />
                  <Text style={[styles.buttonText, styles.primaryButtonText]}>
                    Try Again
                  </Text>
                </TouchableOpacity>

                <TouchableOpacity 
                  style={[styles.button, styles.secondaryButton]}
                  onPress={() => {
                    // Navigate to home screen
                    // You would use your navigation library here
                    console.log('Navigate to home');
                    this.resetError();
                  }}
                >
                  <Home size={20} color={colors.primary} />
                  <Text style={[styles.buttonText, styles.secondaryButtonText]}>
                    Go Home
                  </Text>
                </TouchableOpacity>
              </View>

              {__DEV__ && this.state.error && (
                <View style={styles.devInfo}>
                  <Text style={styles.devInfoTitle}>Development Info:</Text>
                  <Text style={styles.devInfoText}>
                    {this.state.error.stack}
                  </Text>
                </View>
              )}
            </View>
          </ScrollView>
        </View>
      );
    }

    return this.props.children;
  }
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  scrollContent: {
    flexGrow: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 20,
  },
  errorContainer: {
    alignItems: 'center',
    maxWidth: 400,
  },
  iconContainer: {
    marginBottom: 24,
  },
  title: {
    fontSize: 24,
    fontWeight: '700',
    color: colors.text,
    marginBottom: 12,
    textAlign: 'center',
  },
  message: {
    fontSize: 16,
    color: colors.textMuted,
    textAlign: 'center',
    marginBottom: 24,
    lineHeight: 22,
  },
  errorDetails: {
    backgroundColor: colors.white,
    padding: 16,
    borderRadius: 12,
    marginBottom: 24,
    width: '100%',
    borderWidth: 1,
    borderColor: colors.border,
  },
  errorMessage: {
    fontSize: 14,
    color: colors.danger,
    fontFamily: 'monospace',
  },
  actions: {
    flexDirection: 'row',
    gap: 12,
    marginBottom: 24,
  },
  button: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingVertical: 12,
    borderRadius: 12,
    gap: 8,
    minWidth: 120,
    justifyContent: 'center',
  },
  primaryButton: {
    backgroundColor: colors.primary,
  },
  secondaryButton: {
    backgroundColor: colors.white,
    borderWidth: 1,
    borderColor: colors.primary,
  },
  buttonText: {
    fontSize: 16,
    fontWeight: '600',
  },
  primaryButtonText: {
    color: colors.white,
  },
  secondaryButtonText: {
    color: colors.primary,
  },
  devInfo: {
    backgroundColor: '#FFF3E0',
    padding: 16,
    borderRadius: 12,
    width: '100%',
    borderWidth: 1,
    borderColor: '#FFB74D',
  },
  devInfoTitle: {
    fontSize: 14,
    fontWeight: '600',
    color: colors.warning,
    marginBottom: 8,
  },
  devInfoText: {
    fontSize: 12,
    color: colors.textMuted,
    fontFamily: 'monospace',
  },
});

export default ErrorBoundary;