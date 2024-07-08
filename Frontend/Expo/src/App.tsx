// React Native Main File //

// React
import React, { useEffect, useState } from 'react';

// React Native 
import { View, SafeAreaView, Text, StyleSheet, Platform, Dimensions } from 'react-native';

// Expo
import * as ScreenOrientation from 'expo-screen-orientation';

// Components
import BlimpsContainer from './BlimpsContainer';
import Button from './Button';

// Functions
import { useButtonColor } from './AllBlimpsButtonColor'; 

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

  // All Blimps Buttons: Used for Goal Color, Enemy Color, All Auto, All Manual //

  // Goal Color
  const { 
          AllBlimpsButtonStyle: goalColorButtonStyle, 
          buttonColor: goalColorButtonColor, 
          handleClick: handleGoalColorClick 
        } = useButtonColor('orange', 'goal_color');

  // Enemy Color
  const { 
          AllBlimpsButtonStyle: enemyColorButtonStyle, 
          buttonColor: enemyColorButtonColor, 
          handleClick: handleEnemyColorClick 
        } = useButtonColor('blue', 'enemy_color');

  // All Auto
  // ...

  // All Manual
  // ...

  return (
    <SafeAreaView style={styles.container}>

      <SafeAreaView style={styles.topContainer}>
        <Text style={styles.buzzBlimps}>Buzz Blimps</Text>
      </SafeAreaView>

      <SafeAreaView style={styles.buttonContainer}>

          {/* Goal Color Button */}
          <Button 
              blimpName='none' // Blimp Name
              buttonKey='goal_color' // Type of Button
              buttonColor={goalColorButtonColor} // Button Color
              buttonText='Goal' // Text Seen on Button
              buttonStyle={goalColorButtonStyle} // Button Style
              onPress={() => handleGoalColorClick('goal_color')} // On Press Function
          />

          {/* Enemy Color Button */}
          <Button 
              blimpName='none' // Blimp Name
              buttonKey='enemy_color' // Type of Button
              buttonColor={enemyColorButtonColor} // Button Color
              buttonText='Enemy' // Text Seen on Button
              buttonStyle={enemyColorButtonStyle} // Button Style
              onPress={() => handleEnemyColorClick('enemy_color')} // On Press Function
          />

      </SafeAreaView>

      <SafeAreaView style={styles.blimpContainer}>
        {/* Blimps */}
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
