// SidebarMenu.tsx

// React
import React, { useCallback, useRef } from 'react';


// React Native
import { View, Text, Pressable, StyleSheet } from 'react-native';

// React Native Animations
import Animated, { useSharedValue, useAnimatedStyle, withTiming } from 'react-native-reanimated';

// Constants
import { isIOS, isAndroid, isWeb} from '../../Constants/Constants';

// Navigation
import { useNavigation, NavigationProp } from '@react-navigation/native';

// Define the navigation type locally within this file
type RootStackParamList = {
  Buzzstation: undefined;
  Tuning: undefined;
  Telemetry: undefined;
  Logs: undefined;
  Controller: undefined;
  Help: undefined;
};

const SidebarMenu = () => {
  // Use the typed navigation prop directly in the file
  const navigation = useNavigation<NavigationProp<RootStackParamList>>();

  const menuWidth = 150; // Declare menuWidth here
  const sidebarPosition = useSharedValue(-menuWidth); // Start hidden
  const isMenuVisible = useSharedValue(false); // Track visibility state
  const isMenuVisibleRef = useRef(isMenuVisible.value); // Ref to track menu visibility

  // Toggle Sidebar Menu
  const toggleSidebarMenu = useCallback(() => {
    isMenuVisibleRef.current = !isMenuVisible.value; // Update the ref
    isMenuVisible.value = !isMenuVisible.value; // Toggle the visibility

    // Animate the sidebar position
    sidebarPosition.value = withTiming(isMenuVisibleRef.current ? 0 : -menuWidth, { duration: 250 });
  }, [isMenuVisible.value, sidebarPosition, menuWidth]); // Dependencies for the callback

  // Animated style for the sidebar
  
  const animatedStyle = useAnimatedStyle(() => ({
    transform: [{ translateX: sidebarPosition.value }],
  }));

  return (
    <Animated.View style={[styles.sidebarContainer, animatedStyle]}>
      {/* Sidebar Content */}
      <Pressable onPress={() => navigation.navigate('Buzzstation')}>
        <Text style={styles.menuText}>Main</Text>
      </Pressable>
      <Pressable onPress={() => navigation.navigate('Tuning')}>
        <Text style={styles.menuText}>Tuning</Text>
      </Pressable>
      <Pressable onPress={() => navigation.navigate('Telemetry')}>
        <Text style={styles.menuText}>Telemetry</Text>
      </Pressable>
      <Pressable onPress={() => navigation.navigate('Logs')}>
        <Text style={styles.menuText}>Logs</Text>
      </Pressable>
      <Pressable onPress={() => navigation.navigate('Controller')}>
        <Text style={styles.menuText}>Controller</Text>
      </Pressable>
      <Pressable onPress={() => navigation.navigate('Help')}>
        <Text style={styles.menuText}>Help</Text>
      </Pressable>

      {/* Toggle Button for Web */}
      {isWeb && (
      <Pressable
            style={styles.toggleButton}
            onPress={toggleSidebarMenu}
            role='button'
            accessible={true}
            android_disableSound={true}
            android_ripple={{ color: 'transparent' }}
        >
        <Text style={styles.toggleButtonText}>≡</Text>
      </Pressable>
      )}

      {/* Toggle Button for App */}
      {isAndroid || isIOS && (
      <Pressable
            style={styles.toggleButton}
            onPressOut={toggleSidebarMenu}
            role='button'
            accessible={true}
            android_disableSound={true}
            android_ripple={{ color: 'transparent' }}
        >
        <Text style={styles.toggleButtonText}>≡</Text>
      </Pressable>
      )}
    </Animated.View>
  );
};

const styles = StyleSheet.create({
    sidebarContainer: {
      flex: 1,
      opacity: 0.90,
      padding: 20,
      height: '100%',
      alignItems: 'center',
      position: 'absolute', // Make sure it's positioned absolutely
      left: 20, // Align to the left of the screen
      top: 0, // Align to the top of the screen
      zIndex: 10, // Ensure it's on top
    },
    menuText: {
      color: '#ffffff',
      fontSize: 20,
      fontWeight: 'bold',
      marginVertical: 15,
      justifyContent: 'center',
      alignItems: 'center',
      userSelect: 'none',
    },
    toggleButton: {
      position: 'absolute',
      right: isAndroid || isIOS ? -100 : -50,
      top: 0,
      width: isAndroid || isIOS ? 100 : 50,
      height: isAndroid || isIOS ? 100 : 50,
      justifyContent: 'center',
      alignItems: 'center',
    },
    toggleButtonText: {
      color: '#ffffff',
      fontSize: isAndroid || isIOS ? 75 : 50,
      fontWeight: 'bold',
      lineHeight: isAndroid || isIOS ? 75 : 50,
      userSelect: 'none',
    },
  });

export default SidebarMenu;
