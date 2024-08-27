// States.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

export const useStates = () => {
  
    // State Button Text Values
    const [stateValues, setStateValues] = useState<{ [key: string]: string }>({});
  
    // Update State Button Text Value
    useEffect(() => {

        // Event handler for 'update_button_value'
        const handleUpdateButtonValue = (val: { [key: string]: string }) => {
            const receivedName: string = val['name'];
            const receivedButtonKey: string = val['key'];
            const receivedValue: string = val['value'];

            // Catching Blimps
            // 0: Searching
            // 1: Approaching
            // 2: Catching
            // 3: Caught
            // 4: Goal Search
            // 5: Approach Goal
            // 6: Scoring Start
            // 7: Shooting
            // 8: Scored

            // Attacking Blimps
            // False: Searching
            // True: Approaching

            let newValue = 'None'; // Default color 
            if (receivedValue === '0') {
            newValue = 'Searching';
            } else if (receivedValue === '1') {
            newValue = 'Approaching';
            } else if (receivedValue === '2') {
                newValue = 'Catching';
            } else if (receivedValue === '3') {
                newValue = 'Caught';
            } else if (receivedValue === '4') {
                newValue = 'Goal Search';
            } else if (receivedValue === '5') {
                newValue = 'Goal Approach';
            } else if (receivedValue === '6') {
                newValue = 'Scoring Start';
            } else if (receivedValue === '7') {
                newValue = 'Shooting';
            } else if (receivedValue === '8') {
                newValue = 'Scored';
            } else if (receivedValue === 'False') {
                newValue = 'Searching';
            } else if (receivedValue === 'True') {
                newValue = 'Approaching';
            } 
            

            if (receivedButtonKey === 'state_machine') {
                // Update State Values with the new value for the specific blimp
                setStateValues(prevStateValues => ({
                    ...prevStateValues,
                    [receivedName]: newValue,
                }));

                // Testing
                //console.log(`${receivedButtonKey} for ${receivedName} changed to ${receivedValue}`);
            }

        };

        // Listen for 'update_button_color' events
        socket.on('update_button_value', handleUpdateButtonValue);

        // Cleanup to remove the listener when the component is unmounted
        return () => {
            socket.off('update_button_value', handleUpdateButtonValue);
        };

    }, []);

    // State Button Style
    const stateButtonStyle = StyleSheet.create({
      button: {
        width: 140,
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
      stateValues,
      stateButtonStyle,
    };

};
