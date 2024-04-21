// Square Button Component //
// Used for Goal Color and Enemy Color //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { Pressable, StyleSheet, Platform } from 'react-native';

import { socket } from './Constants';

interface SquareButtonProps {
    blimpName: string;
    buttonKey: string;
    buttonColor: string;
}

const SquareButton: React.FC<SquareButtonProps> = ({ blimpName, buttonKey, buttonColor }) => {
    // State to keep track of button color
    const [currentButtonColor, setButtonColor] = useState(buttonColor);

    useEffect(() => {

        // Update Button if correct Key
        socket.on('update_button_color', (val: { [key: string]: string }) => {
            const receivedBlimpName: string = val['blimp'];
            const receivedButtonKey: string = val['key'];
            const receivedButtonColor: string = val['color'];
            
            if (receivedBlimpName === blimpName) {
                if (receivedButtonKey === buttonKey) {
                    setButtonColor(receivedButtonColor);
                }
            }
        });

        // Clean up subscription when component unmounts
        return () => {
            socket.disconnect();
        };
        
    }, []);

    // Handle Button Click
    const handleClick = (buttonKey: string) => {
        socket.emit('toggle_button_color', buttonKey);
        // Testing
        console.log(buttonKey + " button clicked!");
    };

    return (
        <Pressable
        style={[styles.button, { backgroundColor: currentButtonColor }]}
        onPress={() => handleClick(buttonKey)}
        android_disableSound={true}
        android_ripple={{ color: 'transparent' }}
        role='button'
        accessible={true}
        >
        </Pressable>
    );
};

const styles = StyleSheet.create({
    button: {
      width: 40,
      height: 40,
      justifyContent: 'center',
      alignItems: 'center',
      marginVertical: 20,
      marginLeft: Platform.OS === 'ios' ? '15%' : '50%', // Adjust top position based on platform
      marginRight: Platform.OS === 'ios' ? '3%' : '50%', // Adjust top position based on platform
      borderWidth: 2,
      borderColor: 'black'
    }
});

export default SquareButton;