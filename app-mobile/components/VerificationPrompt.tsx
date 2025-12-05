import { TouchableOpacity } from 'react-native';
import { router } from 'expo-router';
import { ThemedView } from '@/components/themed-view';
import { ThemedText } from '@/components/themed-text';
import { useAuth } from '@/contexts/AuthContext';
import Colors from '@/constants/colors';

export default function VerificationPrompt() {
  const { user, isAuthenticated } = useAuth();

  // Only show if user is authenticated but email is not verified
  if (!isAuthenticated || !user || user.emailVerified) {
    return null;
  }

  const handlePress = () => {
    router.push('/verify-email');
  };

  return (
    <ThemedView
      lightColor={Colors.warning}
      darkColor={Colors.warning}
      style={{
        paddingHorizontal: 16,
        paddingVertical: 8,
        borderBottomWidth: 1,
        borderBottomColor: 'rgba(0, 0, 0, 0.1)',
      }}
    >
      <TouchableOpacity 
        onPress={handlePress}
        activeOpacity={0.7}
        style={{
          flexDirection: 'row',
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        <ThemedText
          style={{
            color: Colors.white,
            fontSize: 14,
            fontWeight: '500',
            textAlign: 'center',
            flex: 1,
          }}
        >
          📧 Email verification required - Click to verify
        </ThemedText>
      </TouchableOpacity>
    </ThemedView>
  );
}