// React Native Main File //

// React
import React, { useEffect, useState, Dispatch, SetStateAction } from 'react';

// React Native 
import { View, SafeAreaView, Image, Text, TouchableOpacity, StyleSheet, Platform, Dimensions } from 'react-native';

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

// Helper function to toggle fullscreen on web
function toggleFullScreen(setIsFullScreen: Dispatch<SetStateAction<boolean>>) {
  if (document.fullscreenElement) {
    document.exitFullscreen().then(() => setIsFullScreen(false));
  } else {
    document.documentElement.requestFullscreen().then(() => setIsFullScreen(true));
  }
}

export default function App() {

  useEffect(() => {
    lockOrientation();
  }, []);

  const [isFullScreen, setIsFullScreen] = useState(false);

  // Handle logo press
  const handleLogoPress = () => {
    if (Platform.OS === 'web') {
      toggleFullScreen(setIsFullScreen);
    } else {
      console.log('Logo pressed');
    }
  };

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
      <TouchableOpacity onPress={handleLogoPress}>
        <Image 
          source={require('./assets/buzz_blimps_logo.png')} 
          style={Platform.OS === 'ios' || Platform.OS === 'android' ? styles.imageHalfSize : styles.image} 
        />
      </TouchableOpacity>
      </SafeAreaView>

      <SafeAreaView style={styles.blimpContainer}>
        {/* Blimps */}
        <BlimpsContainer />
      </SafeAreaView>

      <SafeAreaView style={[
        styles.buttonContainer,
        { top: isFullScreen ? '29%' : (Platform.OS === 'android' || Platform.OS === 'ios' ? '35%' : '37%') }
      ]}>

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

        {/* All Auto Button */}
        <Button 
            blimpName='none' // Blimp Name
            buttonKey='all_auto' // Type of Button
            buttonColor='green' // Button Color
            buttonText='All Auto' // Text Seen on Button
            buttonStyle={{
              ...goalColorButtonStyle,
              buttonText: {
                  ...goalColorButtonStyle.buttonText,
                  fontSize: 15,
              },
            }}
            onPress={() => null} // On Press Function
        />

        {/* All Manual Button */}
        <Button 
            blimpName='none' // Blimp Name
            buttonKey='all_manual' // Type of Button
            buttonColor='#E11C1C' // Button Color
            buttonText='All Manual' // Text Seen on Button
            buttonStyle={{
              ...goalColorButtonStyle,
              buttonText: {
                  ...goalColorButtonStyle.buttonText,
                  fontSize: 15,
              },
            }}
            onPress={() => null} // On Press Function
        />

        </SafeAreaView>

    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingTop: 20,
    backgroundColor: 'black', // Set the background color to black
  },
  topContainer: {
    flexDirection: 'row', // Arranges buttons horizontally
    justifyContent: 'center', // Ensures buttons are centered
    alignItems: 'center', // Aligns content in the middle vertically
  },
  buttonContainer: {
    flexDirection: 'column', // Arranges buttons horizontally
    position: 'absolute',
    right: isAndroid || isIOS ? '2%' : '17%',
  },
  blimpContainer: {
    flex: 1,
    justifyContent: 'flex-start',
    alignItems: 'center',
    marginTop: '2%',
  },
  image: {
    resizeMode: 'contain',
  },
  imageHalfSize: {
    width: 624/2, // Half width
    height: 150/2, // Half height
    resizeMode: 'contain',
  },
});
