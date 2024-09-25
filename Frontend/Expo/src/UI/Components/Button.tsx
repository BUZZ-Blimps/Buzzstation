// Button.tsx

// React
import React from 'react';

// React Native
import { Pressable, Text, TextStyle, ViewStyle, View } from 'react-native';

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
  buttonHeartbeat?: number; // Name Last Heartbeat
  onPress: () => void; // Handle Click Function
}

const Button: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonText, buttonStyle, buttonHeartbeat, onPress }) => {

  const normalizedHeartbeat = buttonHeartbeat !== undefined ? buttonHeartbeat : 0;
  const percentageToHighlight = normalizedHeartbeat; // Normalized heartbeat between 0 and 1
  const highlightColor = '#EDA90C'; // Highlight color
  const highlightLength = percentageToHighlight > 0.20 ? Math.round(blimpName.length * percentageToHighlight) : 0; // Highlight if Blimp has not responded after 1 Second (20% of Timeout)

return (
  <Pressable 
      style={[buttonStyle.button, { backgroundColor: buttonColor }]}
      onPress={onPress}
      role='button'
      accessible={true}
      android_disableSound={true}
      android_ripple={{ color: 'transparent' }}
    >
    <Text style={buttonStyle.buttonText}>
      {buttonText.split('').map((char, index) => {
        // Check if the character index is within the highlighted portion
        const isHighlighted = index < highlightLength;
        return (
          <Text key={index} style={{ color: isHighlighted ? highlightColor : 'white' }}>
            {char}
          </Text>
        );
      })}
    </Text>
  </Pressable>
);

};

export default Button;
