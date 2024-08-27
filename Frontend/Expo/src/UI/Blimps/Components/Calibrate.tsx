// Calibrate.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet, Text, View, Pressable } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

// Disable Style Warning
disableStyleWarning();

export const useCalibrate = () => {
    
  // Calibrate Button Colors
  const [calibrateButtonColors, setButtonColors] = useState<{ [key: string]: string }>({});
  
  // Calibrate Button Texts
  const [calibrateButtonTexts, setButtonTexts] = useState<{ [key: string]: string }>({});

  // Update Calibrate Button Color
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

      if (receivedButtonKey === 'calibrate_barometer') {
        // Update calibrateColors with the new color for the specific blimp
        setButtonColors(prevButtonColors => ({
          ...prevButtonColors,
          [receivedName]: newColor,
        }));

        // Testing
        //console.log(`${receivedButtonKey} for ${receivedName} changed to ${receivedButtonColor}`);
      }
    };

    // Listen for 'update_button_color' events
    socket.on('update_button_color', handleUpdateButtonColor);

    // Cleanup to remove the listener when the component is unmounted
    return () => {
      socket.off('update_button_color', handleUpdateButtonColor);
    };

  }, []);

  // Update Blimp Height
  useEffect(() => {
    // Event handler for 'update_button_value'
    const handleUpdateButtonValue = (val: { [key: string]: string }) => {
      const receivedName: string = val['name'];
      const receivedButtonKey: string = val['key'];
      const receivedValue: string = val['value'];

      if (receivedButtonKey === 'height') {
        // Update Button Texts with the new text for the specific blimp
        setButtonTexts(prevButtonTexts => ({
          ...prevButtonTexts,
          [receivedName]: 'Height: ' + receivedValue,
        }));

        // Testing
        //console.log(`${receivedButtonKey} for ${receivedName} changed to ${receivedValue}`);
      }
    };

    // Listen for 'update_button_value' events
    socket.on('update_button_value', handleUpdateButtonValue);

    // Cleanup to remove the listener when the component is unmounted
    return () => {
      socket.off('update_button_value', handleUpdateButtonValue);
    };

  }, []);

  // Handle Calibrate Click
  const handleCalibrateClick = (name: string) => {
    socket.emit('toggle_blimp_calibrate_button_color', name);
  };

  return {
    calibrateButtonColors,
    calibrateButtonTexts,
    CalibrateButton,
    handleCalibrateClick,
  };

};
  
interface CalibrateButtonProps {
    blimpName: string; // Name of the blimp
    buttonKey: string; // Unique key for the button
    buttonColor: string; // Color of the button
    buttonText: string; // Text displayed on the button
    onPress: () => void; // Handle Click Function
}
  
const CalibrateButton: React.FC<CalibrateButtonProps> = ({ blimpName, buttonColor, buttonText, onPress }) => {

  // Button Text Label and Value
  const [label, value] = buttonText.split('Height:');

  // Ensure the value part always has the correct format
  const formattedValue = value ? value.trim().padStart(6, ' ') : '      '; // Add padding for alignment

  return (
    <Pressable
        style={[styles.button, { backgroundColor: buttonColor }]}
        onPress={onPress}
        role='button'
        accessible={true}
        android_disableSound={true} // Optional: Test if needed
        android_ripple={{ color: 'transparent' }} // Optional: Test if needed
      >
      <View style={styles.textContainer}>
          <Text style={[styles.buttonText, styles.buttonTextLabel, , { userSelect: 'none' }]}>
          Height:
          </Text>
          <Text style={[styles.buttonText, styles.buttonTextValue, { userSelect: 'none' }]}>
          {formattedValue}
          </Text>
      </View>
    </Pressable>
  );

};

// Calibrate Button Style
const styles = StyleSheet.create({
  button: {
    width: 130, // Fixed width for button container to fit the maximum text width
    height: 40,
    padding: isAndroid || isIOS ? 9 : 7, // Center Text Vertically
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
    marginHorizontal: 5, // Space between buttons
    justifyContent: 'center',
    alignItems: 'center',
    flexDirection: 'row', // Align items in a row
    flexWrap: 'wrap', // Wrap text if needed
  },
  textContainer: {
    width: '100%', // Fill available width
    flexDirection: 'row', // Align items in a row
    justifyContent: 'center', // Center text horizontally
    alignItems: 'center', // Center text vertically
  },
  buttonText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: isAndroid || isIOS ? 14.25 : 14.25, // Text size
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    fontFamily: 'Arial',
  },
  buttonTextLabel: {
    marginRight: 0, // Small margin to space out label from value
  },
  buttonTextValue: {
    width: 55, // Fixed width for value to ensure consistency
    //textAlign: 'right', // Align value text to the right
  },
});

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
