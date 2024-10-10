// TargetButtonColor.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet, Platform } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

interface Button {
  TargetButtonStyle: any;
  buttonColor: string;
  handleClick: (buttonKey: string) => void;
}

export const useTargetButtonColor = (defaultColor: string, buttonKey: string): Button => {

  // Button Color
  const [buttonColor, setButtonColor] = useState<string>(defaultColor);

  // Update Button Color
  useEffect(() => {

    // Event handler for 'update_button_color'
    const handleUpdateButtonColor = (val: { [key: string]: string }) => {
      const receivedName: string = val['name'];
      const receivedButtonKey: string = val['key'];
      const receivedButtonColor: string = val['color'];

      let newColor = defaultColor; // Default color 
      if (receivedButtonColor === 'orange') {
        newColor = '#FF5D01';
      } else if (receivedButtonColor === 'yellow') {
        newColor = '#DADE00';
      } else if (receivedButtonColor === 'blue') {
        newColor = '#0044A6';
      } else if (receivedButtonColor === 'red') {
        newColor = '#D20F18';
      }

      if (receivedName === 'none') {
        if (receivedButtonKey === buttonKey) {
          setButtonColor(newColor);
          // Testing
          //console.log(buttonKey + " changed to " + receivedButtonColor);
        }
      }
    };

    if (socket) {
      // Listen for 'update_button_color' events
      socket.on('update_button_color', handleUpdateButtonColor);

      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_button_color', handleUpdateButtonColor);
      };
    }

  }, [socket, buttonKey]);

  // Target Button Click
  const handleClick = (buttonKey: string) => {

    if (buttonKey !== 'None') {

      if (socket) {
        socket.emit('toggle_all_blimps_button_color', buttonKey);
      }

      // Testing
      //console.log(buttonKey + " button clicked!");

    }

  };

  // Target Button Style
  const TargetButtonStyle = StyleSheet.create({
    button: {
      width: 100,
      height: 50,
      alignItems: 'center',
      justifyContent: 'center',
      marginVertical: 12,
      borderRadius: 5,
      borderWidth: 2,
      borderColor: 'black',
    },
    buttonText: {
      fontWeight: 'bold',
      color: 'white', // Text color
      fontSize: 18, // Text size
      textAlign: 'center', // Center the text horizontally
      verticalAlign: 'middle', // Center the text vertically
      textShadowColor: 'black', // Outline color
      textShadowOffset: { width: 0, height: 0 }, // Direction of the shadow
      textShadowRadius: isAndroid || isIOS ? 2 : 2.5, // Spread of the shadow
    },
  }); 

  return {
    TargetButtonStyle,
    buttonColor,
    handleClick,
  };

};
