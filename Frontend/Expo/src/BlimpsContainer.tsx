// BlimpsContainer.tsx

// React and React Native
import React from 'react';
import { View, StyleSheet, Platform } from 'react-native';

// Button Component
import Button from './Button';

// Names
import { useNames } from './Names'; // Import the useNames hook

// Modes
import { useModes } from './Modes'; // Import the useModes hook

const BlimpsContainer: React.FC = () => {
  // Names
  const { names, nameColors, nameButtonStyle, handleNameClick } = useNames();

  // States
  // ...

  // Modes
  const { modeColors, modeButtonStyle, handleModeClick } = useModes();

  // Calibrations
  // ...

  // Visions
  // ...

  return (
    <View style={styles.container}>
      {names.length > 0 ? (
        names.map((name, index) => (
          <View key={index} style={styles.buttonRow}>

            {/* Name Button */}
            <Button 
                blimpName={name} // Blimp Name
                buttonKey='name' // Type of Button
                buttonColor={nameColors[name] || 'green'} // Button Color
                buttonText={name} // Text Seen on Button
                buttonStyle={nameButtonStyle} // Button Style
                onPress={() => handleNameClick(name)} // On Press Function
            />

            {/* State Dropdown */}

            {/* Mode Button */}
            <Button 
                blimpName={name} // Blimp Name
                buttonKey='mode' // Type of Button
                buttonColor={modeColors[name] || 'red'} // Button Color
                buttonText={modeColors[name] === 'green' ? 'Auto' : 'Manual'} // Text Seen on Button
                buttonStyle={modeButtonStyle} // Button Style
                onPress={() => handleModeClick(name)} // On Press Function
            />

            {/* Calibrate Button */}

            {/* Vision Button */}
            
          </View>
        ))
      ) : (
        // When no blimps are available, render nothing
        <View />
      )}
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center',
    //backgroundColor: 'black', // Use for testing of columns
  },
  buttonRow: {
    flexDirection: 'row', // Arrange buttons in a row
    justifyContent: 'center',
    alignItems: 'center',
    marginBottom: 10, // Space between rows
    marginHorizontal: 0,
  },
});

export default BlimpsContainer;
