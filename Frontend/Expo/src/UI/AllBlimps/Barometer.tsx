// Barometer.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet, Platform } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

// Float Data Type
import { Float } from 'react-native/Libraries/Types/CodegenTypes';

interface Button {
  BarometerButtonStyle: any;
  buttonColor: string;
  buttonText: string;
  handleClick: () => void;
}

export const useBarometer = (defaultColor: string, defaultText: string): Button => {

  // Button Color
  const [buttonColor, setButtonColor] = useState<string>(defaultColor);

  // Button Text
  const [buttonText, setButtonText] = useState<string>(defaultText);

  // Get Barometer Reading
  useEffect(() => {

    const barometerHandler = (receivedBarometerReading: Float | String) => {
      let barometerReading = receivedBarometerReading;
      let newButtonText = 'Barometer: ' + String(barometerReading);
      setButtonText(newButtonText);
    };

    if (socket) {
      socket.on('barometer_reading', barometerHandler);

      return () => {
        socket.off('barometer_reading', barometerHandler);
      };
    }

  }, [socket]);

  // Get Barometer Button Color
  useEffect(() => {

    // Event handler for 'update_barometer_button_color'
    const handleUpdateBarometerButtonColor = (buttonColor: string) => {

      const receivedButtonColor: string = buttonColor;

      let newColor = defaultColor; // Default color

      if (receivedButtonColor === 'green') {
        newColor = 'green';
      } else if (receivedButtonColor === 'red') {
        newColor = '#E11C1C';
      }

      setButtonColor(newColor);
    };

    if (socket) {
      // Listen for 'update_button_color' events
      socket.on('update_barometer_button_color', handleUpdateBarometerButtonColor);

      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_barometer_button_color', handleUpdateBarometerButtonColor);
      };
    }

  }, [socket]);
  
  // Barometer Button Click
  const handleClick = () => {

      // Testing
      //console.log('Barometer button clicked');

  };

  // Barometer Button Style
  const BarometerButtonStyle = StyleSheet.create({
    button: {
      width: 235,
      height: isAndroid || isIOS ? 50 : 100,
      alignItems: 'center',
      justifyContent: 'center',
      marginVertical: 0,
      marginTop: 5,
      marginLeft: isAndroid || isIOS ? 20 : 20,
      borderRadius: 5,
      borderWidth: 2,
      borderColor: 'black',
    },
    buttonText: {
      fontWeight: 'bold',
      color: 'white', // Text color
      fontSize: isAndroid || isIOS ? 17 : 18, // Text size
      textAlign: 'center', // Center the text horizontally
      verticalAlign: 'middle', // Center the text vertically
      textShadowColor: 'black', // Outline color
      textShadowOffset: { width: 0, height: 0 }, // Direction of the shadow
      textShadowRadius: isAndroid || isIOS ? 2 : 2.5, // Spread of the shadow
    },
  }); 

  return {
    BarometerButtonStyle,
    buttonColor,
    buttonText,
    handleClick,
  };

};
