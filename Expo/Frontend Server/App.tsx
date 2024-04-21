// React Native Main File //

// React
import React, { useEffect, useState } from 'react';

// React Native 
import { View, Text, StyleSheet, Platform, Dimensions } from 'react-native';

// Expo
import * as ScreenOrientation from 'expo-screen-orientation';

// Components
import SquareButton from './SquareButton'; 

export default function App() {

  useEffect(() => {
    if (Platform.OS === 'android' || Platform.OS === 'ios') {
      // Lock the screen orientation to landscape mode
      ScreenOrientation.lockAsync(ScreenOrientation.OrientationLock.LANDSCAPE);
    }
  }, []);

  return (
    <View style={styles.container}>
      <Text style={styles.buzzBlimps}>Buzz Blimps</Text>
      <View style={styles.buttonContainer}>
        <SquareButton blimpName='none' buttonKey='goal_color' buttonColor='orange' />
        <SquareButton blimpName='none' buttonKey='enemy_color' buttonColor='blue' />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    paddingTop: 20,
    position: 'relative', // Position relative to contain absolute elements
  },
  buzzBlimps: {
    fontSize: 40,
    textAlign: 'center',
    position: 'absolute',
    top: Platform.OS === 'ios' ? '5%' : '3%', // Adjust top position based on platform
    left: Platform.OS === 'ios' ? '44%' : '47%', // Position in the middle horizontally
    transform: [{ translateX: -50 }, { translateY: 0 }], // Centering trick
  },
  buttonContainer: {
    flexDirection: 'row',
    position: 'absolute',
    top: Platform.OS === 'ios' ? '1.25%' : '2%', // Adjust top position based on platform
    right: Platform.OS === 'ios' ? '-10%' : '25%', // Adjust top position based on platform
  },
});
