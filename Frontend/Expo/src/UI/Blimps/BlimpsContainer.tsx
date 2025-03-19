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

// Catches
import { useCatches } from './Components/Catches'; // Import the useCatches hook

// Modes
import { useModes } from './Components/Modes'; // Import the useModes hook

// Calibrate
import { useCalibrate } from './Components/Calibrate'; // Import the useCalibrate hook

// Vision
import { useVision } from './Components/Vision'; // Import the useVision hook

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState, AppDispatch } from '../Redux/Store';
import { setCatchIcons, setShootIcons } from '../Redux/States';

const BlimpsContainer: React.FC = () => {

  // Redux Dispatch
  const dispatch: AppDispatch = useDispatch();

  // Names
  const names = useSelector((state: RootState) => state.app.names);

  // Name Button Colors
  const nameColors = useSelector((state: RootState) => state.app.nameColors);

  // Name Last Hearbeats
  const nameLastHeartbeats = useSelector((state: RootState) => state.app.nameLastHeartbeats);
  
  // Name Button Style and Heartbeat and Click Functions
  const { nameButtonStyle, getNormalizedHeartbeat, handleNameClick } = useNames();
  
  // State Values
  const stateValues = useSelector((state: RootState) => state.app.stateValues);

  // Catch counter values
  const catchValues = useSelector((state: RootState) => state.app.catchValues);

  // State Button Style
  const { stateButtonStyle } = useStates();

  // State Button Style
  const { catchButtonStyle } = useCatches();

  // Modes
  const modeColors = useSelector((state: RootState) => state.app.modeColors);
  
  // Mode Button Style and Click Function
  const { modeButtonStyle, handleModeClick } = useModes();

  // Calibrate Colors
  const calibrateColors = useSelector((state: RootState) => state.app.calibrateColors);

  // Calibrate Texts
  const calibrateTexts = useSelector((state: RootState) => state.app.calibrateTexts);

  // Calibrate Button and Click Function
  const { CalibrateButton, handleCalibrateClick } = useCalibrate();

  // Vision Colors
  const visionColors = useSelector((state: RootState) => state.app.visionColors);

  // Vision Button Style and Click Function
  const { visionButtonStyle, handleVisionClick,showCameraStream, CameraStream } = useVision();

  // Catch Icons
  const catchIcons = useSelector((state: RootState) => state.app.catchIcons);

  // Shoot Icons
  const shootIcons = useSelector((state: RootState) => state.app.shootIcons);

  // Toggle Catch Icons
  useEffect(() => {

    const toggleCatchIcon = (data: { name: string; val: boolean }) => {
      const { name: receivedName, val: recievedVal } = data;

      dispatch(setCatchIcons({ [receivedName]: recievedVal }));
    };

    if (socket) {
      socket.on('toggle_catch_icon', toggleCatchIcon);

      return () => {
        socket.off('toggle_catch_icon', toggleCatchIcon);
      };
    }

  }, [socket, catchIcons]);

  // Toggle Shoot Icons
  useEffect(() => {

    const toggleShootIcon = (data: { name: string; val: boolean }) => {
      const { name: receivedName, val: recievedVal } = data;

      dispatch(setShootIcons({ [receivedName]: recievedVal }));
      
    };

    if (socket) {
      socket.on('toggle_shoot_icon', toggleShootIcon);

      return () => {
        socket.off('toggle_shoot_icon', toggleShootIcon);
      };
    }

  }, [socket, shootIcons]);

  const checkCatchIcon = (name: string): boolean => {
    return catchIcons[name] === true; // Returns true if the name is true, otherwise false
  };

  const checkShootIcon = (name: string): boolean => {
    return shootIcons[name] === true; // Returns true if the name is true, otherwise false
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
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
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
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
            },
          }}
          onPress={() => null}
        />

        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Catches'
          buttonColor='black'
          buttonStyle={{
            ...modeButtonStyle,
            button: {
                ...catchButtonStyle.button,
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
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
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
                width: 130,
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
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
                width: 130,
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
                marginTop: isAndroid || isIOS ? -5 : -10,
                marginBottom: isAndroid || isIOS ? -5 : 0,
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

              {/* Catch Counter Button */}
              <Button 
                blimpName={name} // Blimp Name
                buttonKey='catches' // Type of Button
                buttonColor='grey' // Button Color
                buttonText={catchValues[name] || 'None'} // Text Seen on Button
                buttonStyle={stateButtonStyle} // Button Style
                onPress={() => null} // On Press Function
              />

              {/* Mode Button */}
              <Button 
                blimpName={name} // Blimp Name
                buttonKey='mode' // Type of Button
                buttonColor={modeColors[name] || '#E11C1C'} // Button Color (Default: Red)
                buttonText={modeColors[name] === 'green' ? 'Auto' : 'Manual'} // Text Seen on Button
                buttonStyle={modeButtonStyle} // Button Style
                onPress={() => handleModeClick(name)} // On Press Function
              />

              {/* Calibrate Button */}
              <CalibrateButton
                blimpName={name}
                buttonKey='calibrate'
                buttonColor={calibrateColors[name] || '#E11C1C'}
                buttonText={calibrateTexts[name] || 'Height: None'}
                onPress={() => handleCalibrateClick(name)}
              />

              {/* Vision Button */}
              <Button 
                blimpName={name} // Blimp Name
                buttonKey='vision' // Type of Button
                buttonColor={visionColors[name] || 'green'} // Button Color (Default: Green)
                buttonText={visionColors[name] === 'green' ? 'On' : 'Off'} // Text Seen on Button
                buttonStyle={visionButtonStyle} // Button Style
                onPress={() => handleVisionClick(name)} // On Press Function
              />
            </View>
            {showCameraStream[name] && (
              <View style={styles.cameraContainer}>
                <CameraStream name={name} />
             </View>
            )}
            
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
  cameraContainer: {
    marginTop: 5,
    width: '100%',
    alignItems: 'center',
    backgroundColor: 'black', // Optional: gives a background
  },
});

export default BlimpsContainer;
