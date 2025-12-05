import colors from '@/constants/colors';
import { useAuth } from '@/contexts/AuthContext';
import { useHealthData } from '@/contexts/HealthDataContext';
import { useRouter } from 'expo-router';
import { Bluetooth, Edit2, LogOut, Save, X, ChevronDown, Moon, Mail, CheckCircle, AlertCircle } from 'lucide-react-native';
import React, { useEffect, useState } from 'react';
import { ActivityIndicator, Alert, KeyboardAvoidingView, Modal, Platform, ScrollView, StyleSheet, Text, TextInput, TouchableOpacity, View } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

type TabType = 'profile' | 'medical';

// Custom Dropdown Component
interface DropdownOption {
  label: string;
  value: string;
}

interface DropdownProps {
  options: DropdownOption[];
  value: string;
  onValueChange: (value: string) => void;
  placeholder?: string;
  style?: any;
}

const Dropdown: React.FC<DropdownProps> = ({ options, value, onValueChange, placeholder, style }) => {
  const [isVisible, setIsVisible] = useState(false);

  const selectedOption = options.find(option => option.value === value);
  const displayText = selectedOption ? selectedOption.label : placeholder || 'Select an option';

  return (
    <View style={[styles.dropdownContainer, style]}>
      <TouchableOpacity
        style={styles.dropdownButton}
        onPress={() => setIsVisible(!isVisible)}
      >
        <Text style={[styles.dropdownText, !selectedOption && styles.dropdownPlaceholder]}>
          {displayText}
        </Text>
        <ChevronDown
          size={16}
          color={colors.textMuted}
          style={[styles.dropdownIcon, isVisible && styles.dropdownIconRotated]}
        />
      </TouchableOpacity>

      <Modal visible={isVisible} transparent animationType="none">
        <TouchableOpacity
          style={styles.dropdownOverlay}
          onPress={() => setIsVisible(false)}
        >
          <View style={styles.dropdownList}>
            {options.map((option) => (
              <TouchableOpacity
                key={option.value}
                style={styles.dropdownItem}
                onPress={() => {
                  onValueChange(option.value);
                  setIsVisible(false);
                }}
              >
                <Text style={[
                  styles.dropdownItemText,
                  option.value === value && styles.dropdownItemTextSelected
                ]}>
                  {option.label}
                </Text>
                {option.value === value && (
                  <View style={styles.dropdownCheckmark} />
                )}
              </TouchableOpacity>
            ))}
          </View>
        </TouchableOpacity>
      </Modal>
    </View>
  );
};

export default function ProfileScreen() {
  const insets = useSafeAreaInsets();
  const router = useRouter();
  const { userProfile, updateUserProfile, currentData } = useHealthData();
  const { user, signOut, updateUser, isAuthenticated, isLoading, verifyEmail, resendVerificationCode } = useAuth();
  const [activeTab, setActiveTab] = useState<TabType>('profile');
  const [isEditing, setIsEditing] = useState<boolean>(false);
  const [editedProfile, setEditedProfile] = useState(userProfile);
  const [editedUser, setEditedUser] = useState({
    name: user?.name || '',
    email: user?.email || '',
    password: '',
  });

  // Medical conditions and allergies options
  const medicalConditionsOptions: DropdownOption[] = [
    { label: 'None', value: 'none' },
    { label: 'Diabetes', value: 'diabetes' },
    { label: 'Hypertension', value: 'hypertension' },
    { label: 'Asthma', value: 'asthma' },
    { label: 'Heart Disease', value: 'heart_disease' },
    { label: 'Arthritis', value: 'arthritis' },
    { label: 'Other', value: 'other' },
  ];

  const allergiesOptions: DropdownOption[] = [
    { label: 'None', value: 'none' },
    { label: 'Penicillin', value: 'penicillin' },
    { label: 'Pollen', value: 'pollen' },
    { label: 'Nuts', value: 'nuts' },
    { label: 'Shellfish', value: 'shellfish' },
    { label: 'Latex', value: 'latex' },
    { label: 'Other', value: 'other' },
  ];

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

  const handleSave = () => {
    // Update user profile data
    updateUserProfile(editedProfile);

    // Update user data (name and email)
    if (editedUser.name && editedUser.email) {
      updateUser({
        name: editedUser.name,
        email: editedUser.email,
      });
    }

    setIsEditing(false);
    Alert.alert('Success', 'Profile updated successfully');
  };

  const handleCancel = () => {
    setEditedProfile(userProfile);
    setEditedUser({
      name: user?.name || '',
      email: user?.email || '',
      password: '',
    });
    setIsEditing(false);
  };

  return (
    <KeyboardAvoidingView
      style={styles.container}
      behavior={Platform.OS === 'ios' ? 'padding' : 'height'}
      keyboardVerticalOffset={Platform.OS === 'ios' ? 64 : 0}
    >
      <ScrollView contentContainerStyle={styles.contentContainer}>
        <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
          <Text style={styles.headerTitle}>Profile</Text>
          <View style={styles.headerActions}>
            <TouchableOpacity
              style={styles.editButton}
              onPress={() => (isEditing ? handleSave() : setIsEditing(true))}
            >
              {isEditing ? <Save size={20} color={colors.primary} /> : <Edit2 size={20} color={colors.primary} />}
            </TouchableOpacity>
            <TouchableOpacity style={styles.logoutButton} onPress={signOut}>
              <LogOut size={20} color={colors.danger} />
            </TouchableOpacity>
          </View>
        </View>

        <View style={styles.tabContainer}>
          <TouchableOpacity
            style={[styles.tab, activeTab === 'profile' && styles.tabActive]}
            onPress={() => setActiveTab('profile')}
          >
            <Text style={[styles.tabText, activeTab === 'profile' && styles.tabTextActive]}>Profile</Text>
          </TouchableOpacity>
          <TouchableOpacity
            style={[styles.tab, activeTab === 'medical' && styles.tabActive]}
            onPress={() => setActiveTab('medical')}
          >
            <Text style={[styles.tabText, activeTab === 'medical' && styles.tabTextActive]}>Medical Info</Text>
          </TouchableOpacity>
        </View>

        {activeTab === 'profile' ? (
          <View style={styles.content}>
            <View style={styles.profileCard}>
              <Text style={styles.name}>{user?.name || userProfile.name}</Text>
              <View style={styles.emailContainer}>
                <Text style={styles.email}>{user?.email}</Text>
                {user?.emailVerified ? (
                  <View style={styles.verifiedBadge}>
                    <CheckCircle size={16} color={colors.success} />
                    <Text style={styles.verifiedText}>Verified</Text>
                  </View>
                ) : (
                  <TouchableOpacity
                    style={styles.verifyButton}
                    onPress={() => router.push('/verify-email')}
                  >
                    <AlertCircle size={16} color={colors.warning} />
                    <Text style={styles.verifyText}>Verify Email</Text>
                  </TouchableOpacity>
                )}
              </View>
            </View>

            {isEditing ? (
              <View style={styles.section}>
                <Text style={styles.sectionTitle}>Personal Information</Text>
                <View style={styles.inputGroup}>
                  <Text style={styles.inputLabel}>Full Name</Text>
                  <TextInput
                    style={styles.input}
                    value={editedUser.name}
                    onChangeText={(text) => setEditedUser({ ...editedUser, name: text })}
                    placeholder="Enter your full name"
                  />
                </View>
                <View style={styles.inputGroup}>
                  <Text style={styles.inputLabel}>Email</Text>
                  <TextInput
                    style={styles.input}
                    value={editedUser.email}
                    onChangeText={(text) => setEditedUser({ ...editedUser, email: text })}
                    placeholder="Enter your email"
                    keyboardType="email-address"
                    autoCapitalize="none"
                  />
                </View>
                <View style={styles.inputGroup}>
                  <Text style={styles.inputLabel}>Password</Text>
                  <TextInput
                    style={styles.input}
                    value={editedUser.password}
                    onChangeText={(text) => setEditedUser({ ...editedUser, password: text })}
                    placeholder="Enter new password"
                    secureTextEntry
                  />
                </View>
              </View>
            ) : (
              <>
                <View style={styles.section}>
                  <Text style={styles.sectionTitle}>Connected Devices</Text>
                  <View style={styles.deviceCard}>
                    <View style={styles.deviceIcon}>
                      <Bluetooth size={24} color={colors.primary} />
                    </View>
                    <View style={styles.deviceInfo}>
                      <Text style={styles.deviceName}>MedBand Pro X</Text>
                      <Text style={styles.deviceStatus}>Connected • Battery 87%</Text>
                    </View>
                  </View>
                </View>
              </>
            )}
          </View>
        ) : (
          <View style={styles.content}>
            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Information</Text>
              {isEditing ? (
                <View style={styles.inputRow}>
                  <View style={[styles.inputGroup, { flex: 1, marginRight: 8 }]}>
                    <Text style={styles.inputLabel}>Age</Text>
                    <TextInput
                      style={styles.input}
                      value={editedProfile.age?.toString()}
                      onChangeText={(text) => setEditedProfile({ ...editedProfile, age: parseInt(text) || 0 })}
                      placeholder="Age"
                      keyboardType="numeric"
                    />
                  </View>
                  <View style={[styles.inputGroup, { flex: 1, marginLeft: 8 }]}>
                    <Text style={styles.inputLabel}>Gender</Text>
                    <TextInput
                      style={styles.input}
                      value={editedProfile.gender}
                      onChangeText={(text) => setEditedProfile({ ...editedProfile, gender: text })}
                      placeholder="Gender"
                    />
                  </View>
                </View>
              ) : (
                <View style={styles.infoCard}>
                  <View style={[styles.inputGroup, { flex: 1, marginRight: 8 }]}>
                    <Text style={styles.inputLabel}>Age</Text>
                    <Text style={styles.value}>{userProfile.age} years</Text>
                  </View>
                  <View style={[styles.inputGroup, { flex: 1, marginLeft: 8 }]}>
                    <Text style={styles.inputLabel}>Gender</Text>
                    <Text style={styles.value}>{userProfile.gender}</Text>
                  </View>
                </View>
              )}
            </View>

            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Blood Type</Text>
              {isEditing ? (
                <TextInput
                  style={styles.input}
                  value={editedProfile.bloodType}
                  onChangeText={(text) => setEditedProfile({ ...editedProfile, bloodType: text })}
                  placeholder="Blood Type"
                />
              ) : (
                <View style={styles.infoCard}>
                  <Text style={styles.infoValue}>{userProfile.bloodType}</Text>
                </View>
              )}
            </View>

            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Medical Conditions</Text>
              {isEditing ? (
                <Dropdown
                  options={medicalConditionsOptions}
                  value={editedProfile.conditions.length > 0 ? editedProfile.conditions[0] : 'none'}
                  onValueChange={(value) =>
                    setEditedProfile({
                      ...editedProfile,
                      conditions: value === 'none' ? [] : [value === 'other' ? 'Other' : medicalConditionsOptions.find(opt => opt.value === value)?.label || value]
                    })
                  }
                  placeholder="Select medical condition"
                />
              ) : (
                <View style={styles.infoCard}>
                  {userProfile.conditions.length > 0 ? (
                    userProfile.conditions.map((condition, index) => (
                      <View key={index} style={styles.badge}>
                        <Text style={styles.badgeText}>{condition}</Text>
                      </View>
                    ))
                  ) : (
                    <Text style={styles.infoValue}>None</Text>
                  )}
                </View>
              )}
            </View>

            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Allergies</Text>
              {isEditing ? (
                <Dropdown
                  options={allergiesOptions}
                  value={editedProfile.allergies.length > 0 ? editedProfile.allergies[0] : 'none'}
                  onValueChange={(value) =>
                    setEditedProfile({
                      ...editedProfile,
                      allergies: value === 'none' ? [] : [value === 'other' ? 'Other' : allergiesOptions.find(opt => opt.value === value)?.label || value]
                    })
                  }
                  placeholder="Select allergy"
                />
              ) : (
                <View style={styles.infoCard}>
                  {userProfile.allergies.length > 0 ? (
                    userProfile.allergies.map((allergy, index) => (
                      <View key={index} style={[styles.badge, styles.badgeDanger]}>
                        <Text style={[styles.badgeText, styles.badgeTextDanger]}>{allergy}</Text>
                      </View>
                    ))
                  ) : (
                    <Text style={styles.infoValue}>None</Text>
                  )}
                </View>
              )}
            </View>

            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Current Medications</Text>
              {isEditing ? (
                <TextInput
                  style={[styles.input, styles.textArea]}
                  value={editedProfile.medications.join(', ')}
                  onChangeText={(text) =>
                    setEditedProfile({ ...editedProfile, medications: text.split(',').map((s) => s.trim()) })
                  }
                  placeholder="Medications (comma separated)"
                  multiline
                />
              ) : (
                <View style={styles.infoCard}>
                  {userProfile.medications.map((medication, index) => (
                    <Text key={index} style={styles.medicationText}>
                      • {medication}
                    </Text>
                  ))}
                </View>
              )}
            </View>

            {isEditing && (
              <TouchableOpacity style={styles.cancelButton} onPress={handleCancel}>
                <X size={20} color={colors.danger} />
                <Text style={styles.cancelButtonText}>Cancel</Text>
              </TouchableOpacity>
            )}
          </View>
        )}
      </ScrollView>
    </KeyboardAvoidingView>
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
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingBottom: 20,
  },
  headerActions: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  headerTitle: {
    fontSize: 28,
    fontWeight: '700' as const,
    color: colors.text,
  },
  editButton: {
    padding: 8,
    backgroundColor: colors.white,
    borderRadius: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  logoutButton: {
    padding: 8,
    backgroundColor: colors.white,
    borderRadius: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  tabContainer: {
    flexDirection: 'row',
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
  tab: {
    flex: 1,
    paddingVertical: 10,
    alignItems: 'center',
    borderRadius: 8,
  },
  tabActive: {
    backgroundColor: colors.primary,
  },
  tabText: {
    fontSize: 14,
    fontWeight: '600' as const,
    color: colors.textMuted,
  },
  tabTextActive: {
    color: colors.white,
  },
  content: {
    paddingHorizontal: 20,
  },
  profileCard: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 24,
    alignItems: 'center',
    marginBottom: 24,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  name: {
    fontSize: 24,
    fontWeight: '700' as const,
    color: colors.text,
    marginBottom: 8,
    textAlign: 'center',
  },
  info: {
    fontSize: 16,
    color: colors.text,
    marginBottom: 4,
    textAlign: 'center',
  },
  email: {
    fontSize: 14,
    color: colors.textMuted,
    textAlign: 'center',
    marginBottom: 8,
  },
  emailContainer: {
    alignItems: 'center',
  },
  verifiedBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: colors.success + '20',
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 12,
    gap: 4,
  },
  verifiedText: {
    fontSize: 12,
    color: colors.success,
    fontWeight: '600' as const,
  },
  verifyButton: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: colors.warning + '20',
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 12,
    gap: 4,
  },
  verifyText: {
    fontSize: 12,
    color: colors.warning,
    fontWeight: '600' as const,
  },
  section: {
    marginBottom: 24,
  },
  sectionTitle: {
    fontSize: 16,
    fontWeight: '700' as const,
    color: colors.text,
    marginBottom: 12,
  },
  deviceCard: {
    flexDirection: 'row',
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    alignItems: 'center',
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  deviceIcon: {
    width: 48,
    height: 48,
    borderRadius: 24,
    backgroundColor: colors.secondary,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: 12,
  },
  deviceInfo: {
    flex: 1,
  },
  deviceName: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 4,
  },
  deviceStatus: {
    fontSize: 12,
    color: colors.success,
  },
  infoCard: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  infoValue: {
    fontSize: 18,
    fontWeight: '600' as const,
    color: colors.text,
  },
  badge: {
    backgroundColor: colors.secondary,
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 12,
  },
  badgeText: {
    fontSize: 14,
    fontWeight: '600' as const,
    color: colors.primary,
  },
  badgeDanger: {
    backgroundColor: '#FFEBEE',
  },
  badgeTextDanger: {
    color: colors.danger,
  },
  medicationText: {
    fontSize: 14,
    color: colors.text,
    marginBottom: 4,
  },
  inputGroup: {
    marginBottom: 16,
  },
  inputLabel: {
    fontSize: 14,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 8,
  },
  input: {
    backgroundColor: colors.white,
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    color: colors.text,
    borderWidth: 1,
    borderColor: colors.border,
  },
  inputRow: {
    flexDirection: 'row',
    marginBottom: 16,
  },
  textArea: {
    minHeight: 80,
    textAlignVertical: 'top',
  },
  cancelButton: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
    paddingVertical: 12,
    backgroundColor: colors.white,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: colors.danger,
    marginTop: 8,
  },
  infoRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingVertical: 8,
    borderBottomWidth: 1,
    borderBottomColor: colors.border,
  },
  infoLabel: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
  },
  cancelButtonText: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.danger,
  },
  // Dropdown styles
  dropdownContainer: {
    position: 'relative',
    zIndex: 1000,
  },
  dropdownButton: {
    backgroundColor: colors.white,
    borderRadius: 12,
    padding: 16,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    borderWidth: 1,
    borderColor: colors.border,
  },
  dropdownText: {
    flex: 1,
    fontSize: 16,
    color: colors.text,
  },
  dropdownPlaceholder: {
    color: colors.textMuted,
  },
  dropdownIcon: {
    marginLeft: 8,
  },
  dropdownIconRotated: {
    transform: [{ rotate: '180deg' }],
  },
  dropdownOverlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.5)',
  },
  dropdownList: {
    position: 'absolute',
    top: '50%',
    left: 20,
    right: 20,
    backgroundColor: colors.white,
    borderRadius: 12,
    maxHeight: 200,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.15,
    shadowRadius: 8,
    elevation: 5,
  },
  dropdownItem: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 16,
    paddingVertical: 12,
    borderBottomWidth: 1,
    borderBottomColor: colors.border,
  },
  dropdownItemText: {
    flex: 1,
    fontSize: 16,
    color: colors.text,
  },
  dropdownItemTextSelected: {
    color: colors.primary,
    fontWeight: '600' as const,
  },
  dropdownCheckmark: {
    width: 16,
    height: 16,
    borderRadius: 8,
    backgroundColor: colors.primary,
  },
  infoCard2: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 16,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.05,
    shadowRadius: 2,
    elevation: 2,
  },
  row: {
    flexDirection: 'row',
    justifyContent: 'space-between', // Pushes Label left, Value right
    alignItems: 'center',
    paddingVertical: 12,
  },
  line: {
    height: 1,
    backgroundColor: '#f0f0f0',
    width: '100%',
  },
  label: {
    fontSize: 16,
    color: '#666',
  },
  value: {
    fontSize: 16,
    fontWeight: '600',
    color: '#000',
  },
  // Device Status Styles
  statusGrid: {
    flexDirection: 'row',
    gap: 12,
  },
  statusCard: {
    flex: 1,
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    alignItems: 'center',
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  statusIcon: {
    width: 40,
    height: 40,
    borderRadius: 20,
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: 8,
  },
  statusLabel: {
    fontSize: 12,
    color: colors.textMuted,
    marginBottom: 4,
    fontWeight: '500' as const,
  },
  statusValue: {
    fontSize: 14,
    fontWeight: '700' as const,
    textAlign: 'center',
  },
  // Activity Summary Styles
  summaryGrid: {
    flexDirection: 'row',
    gap: 12,
  },
  summaryCard: {
    flex: 1,
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 16,
    alignItems: 'center',
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  summaryIcon: {
    width: 40,
    height: 40,
    borderRadius: 20,
    alignItems: 'center',
    justifyContent: 'center',
    marginBottom: 8,
  },
  summaryLabel: {
    fontSize: 12,
    color: colors.textMuted,
    marginBottom: 4,
    fontWeight: '500' as const,
    textAlign: 'center',
  },
  summaryValue: {
    fontSize: 16,
    fontWeight: '700' as const,
    color: colors.text,
    textAlign: 'center',
  },
  // Sleep Summary Styles
  sleepCard: {
    backgroundColor: colors.white,
    borderRadius: 16,
    padding: 20,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  sleepHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 16,
  },
  sleepTitle: {
    fontSize: 18,
    fontWeight: '700' as const,
    color: colors.text,
    marginLeft: 12,
  },
  sleepStats: {
    flexDirection: 'row',
    justifyContent: 'space-around',
  },
  sleepStat: {
    alignItems: 'center',
  },
  sleepStatValue: {
    fontSize: 24,
    fontWeight: '700' as const,
    color: colors.text,
    marginBottom: 4,
  },
  sleepStatLabel: {
    fontSize: 12,
    color: colors.textMuted,
    fontWeight: '500' as const,
  },
});
