// Vision.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

export const useVision = () => {
  
    // Vision Button Colors
    const [visionColors, setVisionColors] = useState<{ [key: string]: string }>({});
  
    // Update Vision Button Color
    useEffect(() => {

      // Event handler for 'update_button_color'
      const handleUpdateButtonColor = (val: { [key: string]: string }) => {

        const receivedName: string = val['name'];
        const receivedButtonKey: string = val['key'];
        const receivedButtonColor: string = val['color'];

        let newColor = '#E11C1C'; // Default color 
        if (receivedButtonColor === 'red') {
          newColor = '#E11C1C';
        } else if (receivedButtonColor === 'green') {
          newColor = 'green';
        }

        if (receivedButtonKey === 'vision') {
          // Update visionColors with the new color for the specific blimp
          setVisionColors(prevVisionColors => ({
              ...prevVisionColors,
              [receivedName]: newColor,
          }));

          // Testing
          //console.log(`${receivedButtonKey} for ${receivedName} changed to ${receivedButtonColor}`);
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

    }, [socket]);

    // Vision Button Click
    const handleVisionClick = (name: string) => {

      const val = { name: name, key: 'vision'};
      
      if (socket) {
        socket.emit('toggle_blimp_button_color', val);
      }

    };

    // Vision Button Style
    const visionButtonStyle = StyleSheet.create({
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
  
    return {
      visionColors,
      visionButtonStyle,
      handleVisionClick,
    };

};
