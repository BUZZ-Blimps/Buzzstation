// BlimpsContainer.tsx

// React and React Native
import React, { useState, useEffect, useMemo } from 'react';
import { View, StyleSheet, Image } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb } from '../Constants/Constants';

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

// Battery
import { useBattery } from './Components/Battery'; // Import the useBattery hook

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
  const [prevNamesLength, setPrevNamesLength] = useState(names.length);

  // Name Button Colors
  const nameColors = useSelector((state: RootState) => state.app.nameColors);

  // Name Last Heartbeats
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

  // Battery Colors
  const batteryColors = useSelector((state: RootState) => state.app.batteryColors);

  // Battery Texts
  const batteryTexts = useSelector((state: RootState) => state.app.batteryTexts);

  // Battery Button and Click Function
  const { BatteryButton, handleBatteryClick } = useBattery();

  // Vision Colors
  const visionColors = useSelector((state: RootState) => state.app.visionColors);

  // Vision Button Style and Click Function
  const { visionButtonStyle, handleVisionClick, showCameraStream, CameraStream } = useVision();

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

  // This effect will run when the length of names changes.
  useEffect(() => {
    if (names.length !== prevNamesLength) {
      setPrevNamesLength(names.length);
    }
  }, [names]);

  const checkCatchIcon = (name: string): boolean => {
    return catchIcons[name] === true; // Returns true if the name is true, otherwise false
  };

  const checkShootIcon = (name: string): boolean => {
    return shootIcons[name] === true; // Returns true if the name is true, otherwise false
  };

  // Memoizing the names rendering to avoid unnecessary re-renders
  const memoizedBlimps = useMemo(() => {
    return (names.map((name, index) => 
      <View key={index}>
        {/* Main Blimp Buttons Row */}
        <View style={styles.buttonRow}>
          {/* Catch Icon */}
          {checkCatchIcon(name) && (
            <Image
              source={require('../.././assets/catch_icon.png')}
              style={styles.icons}
              resizeMode="contain"
            />
          )}

          {/* Shoot Icon */}
          {checkShootIcon(name) && (
            <Image
              source={require('../.././assets/shoot_icon.png')}
              style={styles.icons}
              resizeMode="contain"
            />
          )}

          {/* Name Button */}
          <Button
            blimpName={name}
            buttonKey='name'
            buttonColor={nameColors[name] || 'green'}
            buttonText={name}
            buttonStyle={nameButtonStyle}
            buttonHeartbeat={getNormalizedHeartbeat(name)}
            onPress={() => handleNameClick(name)}
          />

          {/* State Dropdown */}
          <Button
            blimpName={name}
            buttonKey='state'
            buttonColor='grey'
            buttonText={stateValues[name] || 'None'}
            buttonStyle={stateButtonStyle}
            onPress={() => null}
          />

          {/* Catch Counter Button */}
          <Button
            blimpName={name}
            buttonKey='catches'
            buttonColor='grey'
            buttonText={catchValues[name] !== undefined ? String(catchValues[name]) : 'None'}
            buttonStyle={stateButtonStyle}
            onPress={() => null}
          />

          {/* Battery Button */}
          <BatteryButton
            blimpName={name}
            buttonKey='battery'
            buttonColor={batteryColors[name] || 'grey'}
            buttonText={batteryTexts[name] || 'Battery: N/A'}
            onPress={() => handleBatteryClick(name)}
          />

          {/* Mode Button */}
          <Button
            blimpName={name}
            buttonKey='mode'
            buttonColor={modeColors[name] || '#E11C1C'}
            buttonText={modeColors[name] === 'green' ? 'Auto' : 'Manual'}
            buttonStyle={modeButtonStyle}
            onPress={() => handleModeClick(name)}
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
            blimpName={name}
            buttonKey='vision'
            buttonColor={visionColors[name] || 'green'}
            buttonText={visionColors[name] === 'green' ? 'On' : 'Off'}
            buttonStyle={visionButtonStyle}
            onPress={() => handleVisionClick(name)}
          />
        </View>
        
      </View>
    ));
  }, [names, nameColors, catchIcons, shootIcons, stateValues, catchValues, batteryColors, batteryTexts, modeColors, calibrateColors, calibrateTexts, visionColors, showCameraStream]);

  
  const MemoizedCameraStream = useMemo(() => {
    return (names.map((name, index) => 
      <View key={index}>
          {showCameraStream[name] && (
              <View key={index} style={styles.cameraContainer}>
                <CameraStream name={name} />
              </View>
            )}
        
      </View>
    ));
  },[names,showCameraStream]);
  
  return (
    <View>
      <View style = {styles.streamColumn}>
        
      {/* Left Column for Streams */}
      {MemoizedCameraStream}
       
        
      </View>

      <View style={styles.container}>
      <View style={styles.buttonRow}>
        {/* Header buttons here */}
        
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

        {/* Battery Header */}
        <Button
          blimpName='none'
          buttonKey='none'
          buttonText='Battery'
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

      <View>
        {memoizedBlimps}
      </View>
    </View>
    </View>
    
  );
};

// Blimps Container and Button Row Styles
const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center',
    left: isAndroid || isIOS ? '5%' : '0%',
    position: 'relative'
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: -3,
    marginBottom: 10,
    marginLeft: isAndroid || isIOS ? -50 : 0,
  },
  icons: {
    position: 'relative',
    zIndex: 1,
    width: 500 / 10,
    height: 500 / 10,
    left: isAndroid || isIOS ? '-8.5%' : '-8.5%',
    pointerEvents: 'none',
  },
  cameraContainer: {
    marginTop: 5,
    width: '100%',
    alignItems: 'center',
    backgroundColor: 'black',
    position: "relative",
  },
  streamColumn: {
    width: '40%', // Adjust width as needed
    paddingRight: 10,
    alignItems: 'flex-start', // Left-aligned components in the column
    flexDirection: 'column', // Stack components vertically
    transform: 'translateX(-15vw)', // Position outside the normal flow of the container
    left: 0, // Align to the left outside the container
    zIndex: 10, // Ensure the stream column is above other elements
    position: 'absolute',
  },
  
});

export default BlimpsContainer;
