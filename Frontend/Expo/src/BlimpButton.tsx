// BlimpButton.tsx

// To-Do: Add Update Button Color Socket Receiver

// React
import React, { useState, useEffect } from 'react';

// React Native
import { Pressable, StyleSheet, Platform, Text } from 'react-native';

// SocketIO
import { socket } from './Constants';

// Blimp Names
import { useNames } from './Names'; // Import the useNames hook

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

interface BlimpButtonProps {
  blimpName: string,
  buttonKey: string;
  buttonColor: string;
  buttonText: string;
  buttonWidth: number;
  buttonHeight: number;
  onPress: () => void; // Handle Click Function
}

const BlimpButton: React.FC<BlimpButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonText, buttonWidth, buttonHeight, onPress}) => {

return (
    <Pressable 
        style={[styles.button, { backgroundColor: buttonColor, width: buttonWidth, height: buttonHeight }]}
        onPress={onPress}
        role='button'
    >
        <Text style={styles.buttonText}>{buttonText}</Text>
    </Pressable>
  );
};

const styles = StyleSheet.create({
  button: {
    width: 110,
    height: 40,
    padding: isAndroid || isIOS ? 9 : 7, // Center Text Vertically
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
    marginHorizontal: 5, // Space between buttons
    justifyContent: 'center',
    alignItems: 'center',
  },
  buttonText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 14, // Text size
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
  },
});

export default BlimpButton;
