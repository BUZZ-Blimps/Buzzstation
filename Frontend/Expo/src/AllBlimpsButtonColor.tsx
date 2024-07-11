// AllBlimpsButtonColor.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { View, StyleSheet, Platform } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

interface ButtonColorState {
    AllBlimpsButtonStyle: any; // Replace 'any' with your actual style type if possible
    buttonColor: string;
    handleClick: (buttonKey: string) => void;
}

export const useButtonColor = (defaultColor: string, buttonKey: string): ButtonColorState => {
    const [buttonColor, setButtonColor] = useState<string>(defaultColor);
  
    useEffect(() => {
      // Event handler for 'update_button_color'
      const handleUpdateButtonColor = (val: { [key: string]: string }) => {
        const receivedName: string = val['name'];
        const receivedButtonKey: string = val['key'];
        const receivedButtonColor: string = val['color'];

        if (receivedName === 'none') {
            if (receivedButtonKey === buttonKey) {
                setButtonColor(receivedButtonColor);
                // Testing
                console.log(buttonKey + " changed to " + receivedButtonColor);
            }
        }
      };
  
      // Listen for 'update_button_color' events
      socket.on('update_button_color', handleUpdateButtonColor);
  
      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_button_color', handleUpdateButtonColor);
      };
    }, [buttonKey]);
  
    const handleClick = (buttonKey: string) => {
        if (buttonKey !== 'None') {
            socket.emit('toggle_all_blimps_button_color', buttonKey);
            // Testing
            console.log(buttonKey + " button clicked!");
        }
    };
  
    const AllBlimpsButtonStyle = StyleSheet.create({
      button: {
        width: 100,
        height: 50,
        alignItems: 'center',
        justifyContent: 'center',
        marginVertical: isAndroid || isIOS ? 5 : '14%',
        borderRadius: 5,
        borderWidth: 2,
        borderColor: 'black',
      },
      buttonText: {
          fontWeight: 'bold',
          color: 'white', // Text color
          fontSize: 18, // Text size
          textAlign: 'center', // Center the text
          textAlignVertical: 'center',
          textShadowColor: 'black', // Outline color
          textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
          textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
      },
    }); 
  
    return {
      AllBlimpsButtonStyle,
      buttonColor,
      handleClick,
    };
};
