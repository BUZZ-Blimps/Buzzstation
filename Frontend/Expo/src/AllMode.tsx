// AllMode.tsx

// React and React Native
import { StyleSheet, Platform } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Modes
import { useModes } from './Modes'; // Import the useModes hook

interface Button {
    AllModeButtonStyle: any; // Replace 'any' with your actual style type if possible
    handleClick: (buttonKey: string) => void;
}

export const useAllMode = (defaultColor: string, buttonKey: string): Button => {
    // Modes
    const { modeColors } = useModes();
  
    const handleClick = (buttonKey: string) => {
        if (buttonKey === 'all_auto') {

            // Iterate through the modeColors dictionary
            for (const name in modeColors) {

                if (modeColors.hasOwnProperty(name)) {

                    // Testing
                    //console.log(`Mode for ${name}: ${modeColors[name]}`);
                    // Optionally, you can call handleModeClick or any other function here
                    
                    const val = { name: name, key: 'mode', value: 1};
                    
                    socket.emit('set_blimp_button_value', val);

                }

            }

            // Testing
            //console.log(buttonKey + " button clicked!");

        } else if (buttonKey === 'all_manual') {
            // Iterate through the modeColors dictionary
            for (const name in modeColors) {

                if (modeColors.hasOwnProperty(name)) {

                    // Testing
                    //console.log(`Mode for ${name}: ${modeColors[name]}`);
                    // Optionally, you can call handleModeClick or any other function here
                    
                    const val = { name: name, key: 'mode', value: 0};
                    
                    socket.emit('set_blimp_button_value', val);

                }

            }

            // Testing
            //console.log(buttonKey + " button clicked!");

        }
    };
  
    const AllModeButtonStyle = StyleSheet.create({
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
        fontSize: 15, // Text size
        textAlign: 'center', // Center the text horizontally
        verticalAlign: 'middle', // Center the text vertically
        textShadowColor: 'black', // Outline color
        textShadowOffset: { width: 0, height: 0 }, // Direction of the shadow
        textShadowRadius: isAndroid || isIOS ? 2 : 2.5, // Spread of the shadow
      },
    }); 
  
    return {
      AllModeButtonStyle,
      handleClick,
    };
};
