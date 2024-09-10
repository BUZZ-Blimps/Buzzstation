// Main File //

// React
import React, { useState, useEffect, Dispatch, SetStateAction } from 'react';

// React Native 
import { View, SafeAreaView, Image, Text, Pressable, StyleSheet, Platform, Dimensions } from 'react-native';

// Expo Packages
import * as Updates from 'expo-updates';

// Constants
import { socket, isIOS, isAndroid, isWeb } from './UI/Constants/Constants';

// Components
import Button from './UI/Components/Button';
import BlimpsContainer from './UI/Blimps/BlimpsContainer';
import Controller from './UI/Controller/Controller';

// Functions
import { getUserID } from './UI/Users/UserManager';
import { useBarometer } from './UI/AllBlimps/Barometer';
import { useTargetButtonColor } from './UI/AllBlimps/TargetButtonColor'; 
import { useAllMode } from './UI/AllBlimps/AllMode';

export default function App() {

  // Disable Style Warning
  disableStyleWarning();

  // User ID
  const { userID } = getUserID();

  // Fullscreen State
  const [isFullScreen, setIsFullScreen] = useState(false);

  // Overlay Image
  const [isOverlayImage, setOverlayImage] = useState(false);

  // Handle Overlay Image
  const handleOverlayImage = () => {
    toggleOverlayImage(setOverlayImage);
  };

  // Handle logo press
  const handleLogoPress = () => {
    if (Platform.OS === 'web') {
      toggleFullScreen(setIsFullScreen);
    } else {

      // Testing
      //console.log('Logo pressed');

      // Immediately reload the React Native Bundle
      async function reloadApp () {
        await Updates.reloadAsync();
      }

      reloadApp();

      if (userID) {

        // Inactive User (Reload)
        socket.emit('inactive_user', userID);

      }
      
    }
  };

  // All Blimps Buttons: Used for Barometer, Goal Color, Enemy Color, All Auto, All Manual //

  // Barometer
  const { 
          BarometerButtonStyle: BarometerButtonStyle,
          buttonColor: barometerButtonColor,
          buttonText: barometerButtonText,
          handleClick: handleBarometerClick
        } = useBarometer('#E11C1C', 'Barometer: Disconnected');

  // Goal Color
  const { 
          TargetButtonStyle: goalColorButtonStyle, 
          buttonColor: goalColorButtonColor, 
          handleClick: handleGoalColorClick 
        } = useTargetButtonColor('#FF5D01', 'goal_color');

  // Enemy Color
  const { 
          TargetButtonStyle: enemyColorButtonStyle, 
          buttonColor: enemyColorButtonColor, 
          handleClick: handleEnemyColorClick 
        } = useTargetButtonColor('#0044A6', 'enemy_color');

  // All Auto
  const { 
          AllModeButtonStyle: AllAutoButtonStyle, 
          handleClick: handleAllAutoClick 
        } = useAllMode('green', 'all_auto');

  // All Manual
  const { 
          AllModeButtonStyle: AllManualButtonStyle, 
          handleClick: handleAllManualClick 
        } = useAllMode('#E11C1C', 'all_manual');

  // UI
  return (
    <SafeAreaView style={styles.container}>

      <SafeAreaView style={styles.topContainer}>

        {/* Siderbar Menu Button */}
        <Button 
          blimpName='none' // Blimp Name
          buttonKey='sidebar_menu' // Type of Button
          buttonColor='black' // Button Color
          buttonText='≡' // Text Seen on Button
          buttonStyle={{
            ...goalColorButtonStyle,
            button: {
                ...goalColorButtonStyle.button,
                position: isAndroid || isIOS ?  'relative' : 'absolute',
                left: isAndroid || isIOS ? '0%' : '0%',
                top: isAndroid || isIOS ? '0%' : '-10%',
                marginTop: isAndroid || isIOS ? -5 : -5,
                marginRight: isAndroid || isIOS ? 15 : 0,
                width: 50,
                marginVertical: 0,
            },
            buttonText: {
              ...goalColorButtonStyle.buttonText,
              fontWeight: 'normal',
              fontSize: 40,
            }
          }}
          onPress={() => null} // On Press Function
        />

        {/* Controller Input Display Button */}
        <Controller
          blimpName='none'
          buttonKey='controller'
          buttonColor='black'
          buttonStyle={{
            ...goalColorButtonStyle.button,
            width: isAndroid || isIOS ? 200 : 235,
            height: isAndroid || isIOS ? 40 : 100,
            marginVertical: 0,
            marginTop: 5,
            marginRight: 20,
            borderColor: 'white',
          }}
          onPress={() => handleOverlayImage()}
        />

        <Pressable onPress={handleLogoPress}>

          {/* Buzz Blimps Logo */}
          <Image 
            source={require('./assets/buzz_blimps_logo.png')} 
            resizeMode='contain'
            style={Platform.OS === 'ios' || Platform.OS === 'android' ? styles.imageHalfSize : styles.image} 
          />

        </Pressable>

        {/* Barometer Button */}
        <Button 
          blimpName='none' // Blimp Name
          buttonKey='barometer' // Type of Button
          buttonColor={barometerButtonColor} // Button Color
          buttonText={barometerButtonText} // Text Seen on Button
          buttonStyle={BarometerButtonStyle}
          onPress={() => handleBarometerClick()} // On Press Function
        />

      </SafeAreaView>

      <SafeAreaView style={styles.mainContainer}>

        {/* Blimp Container */}
        <SafeAreaView style={styles.blimpContainer}>

          {/* Blimps */}
          <BlimpsContainer />

        </SafeAreaView>

        {/* To-Do: Make it's own file */}
        {/* All Blimp Buttons Container */}
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

          {/* All Auto Button */}
          <Button 
              blimpName='none' // Blimp Name
              buttonKey='all_auto' // Type of Button
              buttonColor='green' // Button Color
              buttonText='All Auto' // Text Seen on Button
              buttonStyle={AllAutoButtonStyle} // Button Style
              onPress={() => handleAllAutoClick('all_auto')} // On Press Function
          />

          {/* All Manual Button */}
          <Button 
              blimpName='none' // Blimp Name
              buttonKey='all_manual' // Type of Button
              buttonColor='#E11C1C' // Button Color (red)
              buttonText='All Manual' // Text Seen on Button
              buttonStyle={AllManualButtonStyle} // Button Style
              onPress={() => handleAllManualClick('all_manual')} // On Press Function
          />

        </SafeAreaView>

      </SafeAreaView>

      {/* Controller Mapping Overlay Image */}
      {isOverlayImage && (
        <Image
          source={require('./assets/controller_mapping.png')} // Replace with your image path
          style={styles.overlayImage} // style
          resizeMode="contain" // Ensure the image maintains its aspect ratio
        />
      )}

    </SafeAreaView>
  );

};

// Disable Style Warning
function disableStyleWarning() {
  const originalWarn = console.warn;

  console.warn = (message, ...args) => {
    if (typeof message === 'string' && message.includes('"textShadow*" style props are deprecated. Use "textShadow".')) {
      // Suppress the specific warning
      return;
    }
    // Call the original console.warn for other messages
    originalWarn(message, ...args);
  };
}

// Toggle Fullscreen on the Web
function toggleFullScreen(setIsFullScreen: Dispatch<SetStateAction<boolean>>) {
  if (document.fullscreenElement) {
    document.exitFullscreen().then(() => setIsFullScreen(false));
  } else {
    document.documentElement.requestFullscreen().then(() => setIsFullScreen(true));
  }
}

function toggleOverlayImage(setOverlayImage: Dispatch<SetStateAction<boolean>>) {
  setOverlayImage(prevState => !prevState);
}

// Container and Image Styles
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
  mainContainer: {
    flexDirection: 'row',
    justifyContent: 'center', // Center content horizontally
  },
  blimpContainer: {
    justifyContent: 'flex-start',
    alignItems: 'center',
    marginTop: '1.75%',
    marginLeft: isAndroid || isIOS ? '10%' : 100,
  },
  buttonContainer: {
    flexDirection: 'column', // Arranges buttons horizontally
    justifyContent: 'flex-end',
    marginLeft: 5,
  },
  image: {
  },
  imageHalfSize: {
    width: 624/2.6, // Half width
    height: 150/2.6, // Half height
  },
  overlayImage: {
    position: 'absolute',
    zIndex: 1, // Put behind buttons
    width: '100%', // Adjust width to 75% of the parent
    height: '100%', // Adjust height to 75% of the parent
    top: '0%', // Center vertically
    left: isAndroid || isIOS ? '8%' : '0%', // Center horizontally
    pointerEvents: 'none', // Make the image non-clickable
    opacity: 0.75,
  },
});
