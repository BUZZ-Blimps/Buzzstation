// Square Button Component //
// Used for Goal Color and Enemy Color //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { Pressable, StyleSheet, Platform, Text } from 'react-native';

// SocketIO
import { socket } from './Constants';

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

interface ButtonProps {
    blimpName: string;
    buttonKey: string;
    buttonColor: string;
    buttonText: string;
    buttonWidth: number;
    buttonHeight: number;
}

const Button: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonText, buttonWidth, buttonHeight }) => {
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
        if (buttonKey !== 'None') {
            socket.emit('toggle_button_color', buttonKey);
            // Testing
            console.log(buttonKey + " button clicked!");
        }
    };

    return (
        <Pressable 
            style={[styles.button, { backgroundColor: currentButtonColor, width: buttonWidth, height: buttonHeight }]}
            onPress={() => handleClick(buttonKey)}
            android_disableSound={true}
            android_ripple={{ color: 'transparent' }}
            role='button'
            accessible={true}
        >
            <Text style={styles.buttonText}>{buttonText}</Text>
        </Pressable>
    );
};

const styles = StyleSheet.create({
    button: {
      alignItems: 'center',
      marginVertical: 20,
      marginRight: isAndroid || isIOS ? '1%' : '1%', // Adjust top position based on platform
      padding: isAndroid || isIOS ? 10 : 7, // Center Text Vertically
      borderRadius: 5,
      borderWidth: 2,
      borderColor: 'black',
    },
    buttonText: {
        fontWeight: 'bold',
        color: 'white', // Text color
        fontSize: 20, // Text size
        textAlign: 'center', // Center the text
        textShadowColor: 'black', // Outline color
        textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
        textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
    },
});

export default Button;