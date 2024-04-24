// React Native Main File //

// React
import React, { useEffect, useState } from 'react';

// React Native 
import { View, SafeAreaView, Text, StyleSheet, Platform, Dimensions } from 'react-native';

// Expo
import * as ScreenOrientation from 'expo-screen-orientation';

// Components
import Button from './Button'; 
import BlimpsContainer from './BlimpsContainer'; 

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Lock the screen orientation to landscape mode on mobile platforms
function lockOrientation() {
  if (isAndroid || isIOS) {
    ScreenOrientation.lockAsync(ScreenOrientation.OrientationLock.LANDSCAPE);
  }
}

export default function App() {

  useEffect(() => {
    lockOrientation();
  }, []);

  return (
    <SafeAreaView style={styles.container}>

      <SafeAreaView style={styles.topContainer}>
        <Text style={styles.buzzBlimps}>Buzz Blimps</Text>
      </SafeAreaView>

      <SafeAreaView style={styles.buttonContainer}>
          <Button blimpName='none' buttonKey='goal_color' buttonColor='orange' buttonText='Goal' buttonWidth={100} buttonHeight={50} />
          <Button blimpName='none' buttonKey='enemy_color' buttonColor='blue' buttonText='Enemy' buttonWidth={100} buttonHeight={50} />
      </SafeAreaView>

      <SafeAreaView style={styles.blimpContainer}>
        <BlimpsContainer />
      </SafeAreaView>

    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingTop: 20,
  },
  buzzBlimps: {
    fontSize: 40,
    position: 'absolute', // Enables absolute positioning
    top: 15, // Distance from the top edge
    left: isAndroid || isIOS ? '41%' : '47%', // Positions the text at the horizontal center
    transform: [{ translateX: -50 }], // Adjusts position to ensure exact centering
    fontWeight: 'bold',
    color: 'gold', // Text color
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 2, height: 2 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 1 : 2.5, // Spread of the shadow
  },
  topContainer: {
    flexDirection: 'row', // Arranges buttons horizontally
    justifyContent: 'center', // Ensures buttons are at the end of the row
    alignItems: 'center', // Aligns content in the middle vertically
  },
  buttonContainer: {
    flexDirection: 'row', // Arranges buttons horizontally
    justifyContent: 'flex-end', // Ensures buttons are at the end of the row
    alignItems: 'center', // Aligns content in the middle vertically
  },
  blimpContainer: {
    flex: 1,
    justifyContent: 'flex-start',
    alignItems: 'flex-start',
    marginLeft: '2%',
  },
});
