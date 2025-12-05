import React, { useState } from 'react';
import { View, Text, StyleSheet, ScrollView, TouchableOpacity, Switch } from 'react-native';
import { Stack } from 'expo-router';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import {
  Globe,
  Moon,
  Shield,
  MapPin,
  Bluetooth,
  Bell,
  Info,
  HelpCircle,
  ChevronRight,
  Wrench,
} from 'lucide-react-native';
import colors from '@/constants/colors';

export default function SettingsScreen() {
  const insets = useSafeAreaInsets();
  const [locationEnabled, setLocationEnabled] = useState<boolean>(true);
  const [bluetoothEnabled, setBluetoothEnabled] = useState<boolean>(true);
  const [notificationsEnabled, setNotificationsEnabled] = useState<boolean>(true);

  const SettingRow = ({
    icon: Icon,
    title,
    subtitle,
    onPress,
    showChevron = true,
  }: {
    icon: React.ComponentType<{ size: number; color: string }>;
    title: string;
    subtitle?: string;
    onPress?: () => void;
    showChevron?: boolean;
  }) => (
    <TouchableOpacity style={styles.settingRow} onPress={onPress} disabled={!onPress}>
      <View style={styles.settingLeft}>
        <View style={styles.iconContainer}>
          <Icon size={20} color={colors.primary} />
        </View>
        <View style={styles.settingText}>
          <Text style={styles.settingTitle}>{title}</Text>
          {subtitle && <Text style={styles.settingSubtitle}>{subtitle}</Text>}
        </View>
      </View>
      {showChevron && <ChevronRight size={20} color={colors.textMuted} />}
    </TouchableOpacity>
  );

  const SettingToggle = ({
    icon: Icon,
    title,
    subtitle,
    value,
    onValueChange,
  }: {
    icon: React.ComponentType<{ size: number; color: string }>;
    title: string;
    subtitle?: string;
    value: boolean;
    onValueChange: (value: boolean) => void;
  }) => (
    <View style={styles.settingRow}>
      <View style={styles.settingLeft}>
        <View style={styles.iconContainer}>
          <Icon size={20} color={colors.primary} />
        </View>
        <View style={styles.settingText}>
          <Text style={styles.settingTitle}>{title}</Text>
          {subtitle && <Text style={styles.settingSubtitle}>{subtitle}</Text>}
        </View>
      </View>
      <Switch
        value={value}
        onValueChange={onValueChange}
        trackColor={{ false: colors.border, true: colors.primaryLight }}
        thumbColor={value ? colors.primary : colors.white}
      />
    </View>
  );

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
        <View style={[styles.header, { paddingTop: insets.top + 20 }]}>
          <Text style={styles.headerTitle}>Settings</Text>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Preferences</Text>
          <View style={styles.card}>
            <SettingRow icon={Globe} title="Language" subtitle="English" onPress={() => console.log('Language')} />
            <View style={styles.divider} />
            <SettingRow icon={Moon} title="Theme" subtitle="Light" onPress={() => console.log('Theme')} />
            <View style={styles.divider} />
            <SettingRow icon={Shield} title="Privacy" subtitle="Manage your data" onPress={() => console.log('Privacy')} />
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Permissions</Text>
          <View style={styles.card}>
            <SettingToggle
              icon={MapPin}
              title="Location"
              subtitle="Allow location access"
              value={locationEnabled}
              onValueChange={setLocationEnabled}
            />
            <View style={styles.divider} />
            <SettingToggle
              icon={Bluetooth}
              title="Bluetooth"
              subtitle="Connect to devices"
              value={bluetoothEnabled}
              onValueChange={setBluetoothEnabled}
            />
            <View style={styles.divider} />
            <SettingToggle
              icon={Bell}
              title="Notifications"
              subtitle="Health alerts & reminders"
              value={notificationsEnabled}
              onValueChange={setNotificationsEnabled}
            />
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Support</Text>
          <View style={styles.card}>
            <SettingRow
              icon={Info}
              title="About"
              subtitle="Version 1.0.0"
              onPress={() => console.log('About')}
            />
            <View style={styles.divider} />
            <SettingRow
              icon={HelpCircle}
              title="Help & Support"
              subtitle="Contact us"
              onPress={() => console.log('Help')}
            />
            <View style={styles.divider} />
            <SettingRow
              icon={Wrench}
              title="Troubleshooting"
              subtitle="Connection issues"
              onPress={() => console.log('Troubleshooting')}
            />
          </View>
        </View>

        <View style={styles.footer}>
          <Text style={styles.footerText}>MedBand Companion App</Text>
          <Text style={styles.footerSubtext}>Made with care for your health</Text>
        </View>
    </ScrollView>
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
  header: {
    paddingHorizontal: 20,
    paddingBottom: 20,
  },
  headerTitle: {
    fontSize: 28,
    fontWeight: '700' as const,
    color: colors.text,
  },
  section: {
    marginBottom: 24,
    paddingHorizontal: 20,
  },
  sectionTitle: {
    fontSize: 14,
    fontWeight: '700' as const,
    color: colors.textMuted,
    marginBottom: 12,
    textTransform: 'uppercase' as const,
    letterSpacing: 0.5,
  },
  card: {
    backgroundColor: colors.white,
    borderRadius: 16,
    shadowColor: colors.shadow,
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 8,
    elevation: 3,
  },
  settingRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
  },
  settingLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
  },
  iconContainer: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: colors.secondary,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: 12,
  },
  settingText: {
    flex: 1,
  },
  settingTitle: {
    fontSize: 16,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 2,
  },
  settingSubtitle: {
    fontSize: 12,
    color: colors.textMuted,
  },
  divider: {
    height: 1,
    backgroundColor: colors.border,
    marginLeft: 68,
  },
  footer: {
    alignItems: 'center',
    marginTop: 24,
    paddingHorizontal: 20,
  },
  footerText: {
    fontSize: 14,
    fontWeight: '600' as const,
    color: colors.text,
    marginBottom: 4,
  },
  footerSubtext: {
    fontSize: 12,
    color: colors.textMuted,
  },
});
