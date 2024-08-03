// BlimpsContainer.tsx

// React and React Native
import React from 'react';
import { View, StyleSheet, Platform } from 'react-native';

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Button Component
import Button from './Button';

// Names
import { useNames } from './Names'; // Import the useNames hook

// States
import { useStates } from './States'; // Import the useModes hook

// Modes
import { useModes } from './Modes'; // Import the useModes hook

// Calibrate
//import { useCalibrate } from './Calibrate'; // Import the useVision hook

// Vision
import { useVision } from './Vision'; // Import the useVision hook

const BlimpsContainer: React.FC = () => {
  // Names
  const { names, nameColors, nameButtonStyle, handleNameClick } = useNames();

  // States
  const { stateValues, stateButtonStyle } = useStates();

  // Modes
  const { modeColors, modeButtonStyle, handleModeClick } = useModes();

  // Calibrations
  // ...

  // Visions
  const { visionColors, visionButtonStyle, handleVisionClick } = useVision();

  return (
    <View style={styles.container}>
      {/* Row of 5 Column Header */}
      <View style={styles.buttonRow}>
        {/* Blimps Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Blimps'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...modeButtonStyle.button,
                marginTop: -10,
            },
          }}
          onPress={() => null}
        />
        {/* State Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='State'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...stateButtonStyle.button,
                marginTop: -10,
            },
          }}
          onPress={() => null}
        />
        {/* Mode Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Mode'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...modeButtonStyle.button,
                marginTop: -10,
            },
          }}
          onPress={() => null}
        />
        {/* Calibrate Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Calibrate'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...modeButtonStyle.button,
                marginTop: -10,
                width: 130, // Example new width
            },
          }}
          onPress={() => null}
        />
        {/* Vision Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Vision'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...modeButtonStyle.button,
                marginTop: -10,
            },
          }}
          onPress={() => null}
        />

      </View>
      
      {names.length > 0 ? (
        names.map((name, index) => (
          <View key={index}>

            {/* Main Blimp Buttons Row */}
            <View style={styles.buttonRow}>

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
              {/* Placeholder Button Here: */}
              <Button 
                  blimpName={name} // Blimp Name
                  buttonKey='state' // Type of Button
                  buttonColor='grey' // Button Color
                  buttonText={stateValues[name] || 'None'} // Text Seen on Button
                  buttonStyle={stateButtonStyle} // Button Style
                  onPress={() => null} // On Press Function
              />

              {/* Mode Button */}
              <Button 
                  blimpName={name} // Blimp Name
                  buttonKey='mode' // Type of Button
                  buttonColor={modeColors[name] || 'red'} // Button Color (Default: Red)
                  buttonText={modeColors[name] === 'green' ? 'Auto' : 'Manual'} // Text Seen on Button
                  buttonStyle={modeButtonStyle} // Button Style
                  onPress={() => handleModeClick(name)} // On Press Function
              />

              {/* Calibrate Button */}
              {/* Placeholder Button Here: */}
              <Button 
                  blimpName={name} // Blimp Name
                  buttonKey='calibrate' // Type of Button
                  buttonColor='#E11C1C' // Button Color
                  buttonText='Height: 0.00m' // Text Seen on Button
                  buttonStyle={{
                    ...modeButtonStyle,
                    button: {
                        ...modeButtonStyle.button,
                        width: 130, // Example new width
                    },
                  }}
                  onPress={() => null} // On Press Function
              />

              {/* Vision Button */}
              {/* Placeholder Button Here: */}
              <Button 
                  blimpName={name} // Blimp Name
                  buttonKey='vision' // Type of Button
                  buttonColor={visionColors[name] || 'green'} // Button Color (Default: Green)
                  buttonText={visionColors[name] === 'green' ? 'On' : 'Off'} // Text Seen on Button
                  buttonStyle={visionButtonStyle} // Button Style
                  onPress={() => handleVisionClick(name)} // On Press Function
              />
            </View>
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
    marginTop: -3,
    marginBottom: 10, // Space between rows
    marginLeft: isAndroid || isIOS ? -50 : 0,
  },
});

export default BlimpsContainer;
