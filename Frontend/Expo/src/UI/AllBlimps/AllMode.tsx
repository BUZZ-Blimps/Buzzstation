// AllMode.tsx

// React and React Native
import { StyleSheet } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

// Modes
import { useModes } from '../Blimps/Components/Modes';

interface Button {
    AllModeButtonStyle: any;
    handleClick: (buttonKey: string) => void;
}

export const useAllMode = (defaultColor: string, buttonKey: string): Button => {

    // Modes
    const { modeButtonColors } = useModes();
  
    // All Mode Button Click
    const handleClick = (buttonKey: string) => {
        if (buttonKey === 'all_auto') {

            // Iterate through the modeButtonColors dictionary
            for (const name in modeButtonColors) {

                if (modeButtonColors.hasOwnProperty(name)) {

                    // Testing
                    //console.log(`Mode for ${name}: ${modeButtonColors[name]}`);
                    // Optionally, you can call handleModeClick or any other function here
                    
                    const val = { name: name, key: 'mode', value: 1};
                    
                    socket.emit('set_blimp_button_value', val);

                }

            }

            // Testing
            //console.log(buttonKey + " button clicked!");

        } else if (buttonKey === 'all_manual') {
            // Iterate through the modeButtonColors dictionary
            for (const name in modeButtonColors) {

                if (modeButtonColors.hasOwnProperty(name)) {

                    // Testing
                    //console.log(`Mode for ${name}: ${modeButtonColors[name]}`);
                    // Optionally, you can call handleModeClick or any other function here
                    
                    const val = { name: name, key: 'mode', value: 0};
                    
                    socket.emit('set_blimp_button_value', val);

                }

            }

            // Testing
            //console.log(buttonKey + " button clicked!");

        }
    };
  
    // All Mode Button Style
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