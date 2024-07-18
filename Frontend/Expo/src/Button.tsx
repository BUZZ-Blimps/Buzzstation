// Button.tsx

// React
import React from 'react';

// React Native
import { Pressable, Text, TextStyle, ViewStyle } from 'react-native';

interface ButtonStyle {
  button: ViewStyle; // Style for the button container
  buttonText: TextStyle; // Style for the text inside the button
}

interface ButtonProps {
  blimpName: string; // Name of the blimp
  buttonKey: string; // Unique key for the button
  buttonColor: string; // Color of the button
  buttonText: string; // Text displayed on the button
  buttonStyle: ButtonStyle; // Text Style for the button
  onPress: () => void; // Handle Click Function
}

const Button: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonText, buttonStyle, onPress }) => {

return (
    <Pressable 
        style={[buttonStyle.button, { backgroundColor: buttonColor }]}
        onPress={onPress}
        role='button'
        accessible={true}
        android_disableSound={true}
        android_ripple={{ color: 'transparent' }}
    >
      <Text selectable={false} style={buttonStyle.buttonText}>{buttonText}</Text>
    </Pressable>
  );
};

export default Button;
