// BlimpsContainer.tsx

// React and React Native
import React, { useState, useEffect } from 'react';
import { View, StyleSheet, Image } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

// Button Component
import Button from '../Components/Button';

// Names
import { useNames } from './Components/Names'; // Import the useNames hook

// States
import { useStates } from './Components/States'; // Import the useModes hook

// Modes
import { useModes } from './Components/Modes'; // Import the useModes hook

// Calibrate
import { useCalibrate } from './Components/Calibrate'; // Import the useCalibrate hook

// Vision
import { useVision } from './Components/Vision'; // Import the useVision hook

const BlimpsContainer: React.FC = () => {

  // Names
  const { names, nameColors, nameButtonStyle, getNormalizedHeartbeat, handleNameClick } = useNames();

  // States
  const { stateValues, stateButtonStyle } = useStates();

  // Modes
  const { modeButtonColors, modeButtonStyle, handleModeClick } = useModes();

  // Calibrations
  const { calibrateButtonColors, calibrateButtonTexts, CalibrateButton, handleCalibrateClick } = useCalibrate();

  // Visions
  const { visionColors, visionButtonStyle, handleVisionClick } = useVision();

  // Catch Icon
  const [isCatchIcon, setCatchIcon] = useState<{ [key: string]: boolean }>({});

  // Catch Icon
  const [isShootIcon, setShootIcon] = useState<{ [key: string]: boolean }>({});

  // Toggle Catch Icon
  useEffect(() => {

    const toggleCatchIcon = (data: { name: string; val: boolean }) => {
      const { name: receivedName, val: recievedVal } = data;

      setCatchIcon((prevVals) => ({
        ...prevVals,
        [receivedName]: recievedVal,
      }));
    };

    socket.on('toggle_catch_icon', toggleCatchIcon);

    return () => {
      socket.off('toggle_catch_icon', toggleCatchIcon);
    };

  }, [isCatchIcon]);

  // Shoot Catch Icon
  useEffect(() => {

    const toggleShootIcon = (data: { name: string; val: boolean }) => {
      const { name: receivedName, val: recievedVal } = data;

      setShootIcon((prevVals) => ({
        ...prevVals,
        [receivedName]: recievedVal,
      }));
    };

    socket.on('toggle_shoot_icon', toggleShootIcon);

    return () => {
      socket.off('toggle_shoot_icon', toggleShootIcon);
    };

  }, [isShootIcon]);

  const checkCatchIcon = (name: string): boolean => {
    return isCatchIcon[name] === true; // Returns true if the name is true, otherwise false
  };

  const checkShootIcon = (name: string): boolean => {
    return isShootIcon[name] === true; // Returns true if the name is true, otherwise false
  };

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
                width: 130,
            },
          }}
          onPress={() => null}
        />

        {/* Vision Header */}
        {/* <Button
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
        /> */}

      </View>
      
      {names.length > 0 ? (
        names.map((name, index) => (
          <View key={index}>

            {/* Main Blimp Buttons Row */}
            <View style={styles.buttonRow}>

              {/* Catch Icon */}
              {checkCatchIcon(name) && (
                <Image
                  source={require('../.././assets/catch_icon.png')} // Replace with your image path
                  style={styles.icons} // style
                  resizeMode="contain" // Ensure the image maintains its aspect ratio
                />
              )}

              {/* Shoot Icon */}
              {checkShootIcon(name) && (
                <Image
                  source={require('../.././assets/shoot_icon.png')} // Replace with your image path
                  style={styles.icons} // style
                  resizeMode="contain" // Ensure the image maintains its aspect ratio
                />
              )}

              {/* Name Button */}
              <Button 
                blimpName={name} // Blimp Name
                buttonKey='name' // Type of Button
                buttonColor={nameColors[name] || 'green'} // Button Color
                buttonText={name} // Text Seen on Button
                buttonStyle={nameButtonStyle} // Button Style
                buttonHeartbeat={getNormalizedHeartbeat(name)} // Pass normalized heartbeat value here
                onPress={() => handleNameClick(name)} // On Press Function
              />

              {/* State Dropdown */}
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
                buttonColor={modeButtonColors[name] || '#E11C1C'} // Button Color (Default: Red)
                buttonText={modeButtonColors[name] === 'green' ? 'Auto' : 'Manual'} // Text Seen on Button
                buttonStyle={modeButtonStyle} // Button Style
                onPress={() => handleModeClick(name)} // On Press Function
              />

              {/* Calibrate Button */}
              <CalibrateButton
                blimpName={name}
                buttonKey='calibrate'
                buttonColor={calibrateButtonColors[name] || '#E11C1C'}
                buttonText={calibrateButtonTexts[name] || 'Height: None'}
                onPress={() => handleCalibrateClick(name)}
              />

              {/* Vision Button */}
              {/* <Button 
                blimpName={name} // Blimp Name
                buttonKey='vision' // Type of Button
                buttonColor={visionColors[name] || 'green'} // Button Color (Default: Green)
                buttonText={visionColors[name] === 'green' ? 'On' : 'Off'} // Text Seen on Button
                buttonStyle={visionButtonStyle} // Button Style
                onPress={() => handleVisionClick(name)} // On Press Function
              /> */}

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

// Blimps Container and Button Row Styles
const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center',
    left: isAndroid || isIOS ? '5%' : '0%',
    //backgroundColor: 'white', // Use for testing of columns
  },
  buttonRow: {
    flexDirection: 'row', // Arrange buttons in a row
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: -3,
    marginBottom: 10, // Space between rows
    marginLeft: isAndroid || isIOS ? -50 : 0,
  },
  icons: {
    position: 'absolute',
    zIndex: 1, // Put behind buttons
    width: 500/10, // Adjust width to 75% of the parent
    height: 500/10, // Adjust height to 75% of the parent
    left: isAndroid || isIOS ? '-8.5%' : '-8.5%', // Center horizontally
    pointerEvents: 'none', // Make the image non-clickable
  },
});

export default BlimpsContainer;
