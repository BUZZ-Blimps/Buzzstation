// Controller.tsx

// React
import React, { useEffect, useState, useRef, useCallback } from 'react';

// React Native
import { View, Pressable, StyleSheet, Platform, ViewStyle, TextStyle } from 'react-native';

// React Native Animations
import Animated, { Easing, useSharedValue, withTiming, useAnimatedStyle } from 'react-native-reanimated';

// SocketIO
import { socket } from './Constants';

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Web
const isWeb = Platform.OS === 'web';

// Names
import { useNames } from './Names';

// Gamepads
import { useGamepads } from "react-gamepads";

interface ButtonStyle {
    button: ViewStyle; // Style for the button container
    buttonText: TextStyle; // Style for the text inside the button
}

interface ButtonProps {
    blimpName: string; // Name of the blimp
    buttonKey: string; // Unique key for the button
    buttonColor: string; // Color of the button
    buttonStyle: ButtonStyle; // Style for the button
    onPress: () => void; // Handle Click Function
}

const Controller: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonStyle, onPress }) => {

    // Dot Values
    const leftDotX = useSharedValue(50);
    const leftDotY = useSharedValue(50);
    const rightDotX = useSharedValue(50);
    const rightDotY = useSharedValue(50);

    // Names, Name Colors, and UserIDs
    const { names, nameColors, userID } = useNames();

    // GamePads
    const [gamepads, setGamepads] = useState<{ [index: number]: Gamepad }>({});

    // First Gamepad
    const firstGamepad = Object.values(gamepads)[0];

    // Axes States
    const prevAxesRef = useRef([0, 0, 0, 0]);

    // Button States
    const [buttonStates, setButtonStates] = useState<{ [key: string]: boolean }>({});
    const prevButtonStatesRef = useRef<{ [key: string]: boolean }>({});

    // Get Gamepads for Web
    if (isWeb) {
      useGamepads((gamepads) => setGamepads(gamepads));
    }

    // Update Buttons
    const updateButtons = useCallback((gamepad: Gamepad) => {

      const newButtonStates: { [key: string]: boolean } = {};

      gamepad.buttons.forEach((button, index) => {
        newButtonStates[`button${index}`] = button.pressed;
      });
  
      const prevButtonStates = prevButtonStatesRef.current;
  
      let stateChanged = false;
      for (let key in newButtonStates) {
        if (prevButtonStates[key] !== newButtonStates[key]) {
          stateChanged = true;
          if (prevButtonStates[key] && !newButtonStates[key]) {

            let count = 0;

            // Handle the button release event here
            for (let index = 0; index < names.length; index++) {
              const name = names[index];
              if (nameColors[name] === '#006FFF') {
                let val: { name?: string; button?: string; userID?: string} = {};
                val['name'] = name;
                val['button'] = key;
                val['userID'] = String(userID);
                
                // Emit to Backend
                socket.emit('blimp_button', val);

                // Increase Count
                count++;
              }
            }

            // No Blimps Connected
            if (count === 0) {
              let val: { button?: string; userID?: string} = {};
              val['button'] = key;
              val['userID'] = String(userID);
              socket.emit('nonblimp_button', val);
            }

          }
        }
      }
  
      if (stateChanged) {
        setButtonStates(newButtonStates);
        prevButtonStatesRef.current = newButtonStates;
      }
    }, [names, nameColors]);

    // Update Joysticks
    const updateJoysticks = useCallback((gamepad: Gamepad) => {
      const { axes } = gamepad;
      const [leftStickX, leftStickY, rightStickX, rightStickY] = axes;

      const deadZone = 0.1;
      const deadZoneApplied = (value: number) => (Math.abs(value) < deadZone ? 0 : value);
      const clampedAxes = [
          parseFloat(deadZoneApplied(leftStickX).toFixed(2)),
          parseFloat(deadZoneApplied(leftStickY).toFixed(2)),
          parseFloat(deadZoneApplied(rightStickX).toFixed(2)),
          parseFloat(deadZoneApplied(rightStickY).toFixed(2)),
      ];

      const prevAxes = prevAxesRef.current;
      const axesChanged = !clampedAxes.every((value, index) => value === prevAxes[index]);

      if (axesChanged) {
          prevAxesRef.current = clampedAxes;
          for (let index = 0; index < names.length; index++) {
              const name = names[index];
              if (nameColors[name] === '#006FFF') {
                  let val: { name?: string; axes?: number[] } = {};
                  val['name'] = name;
                  val['axes'] = clampedAxes;
                  socket.emit('motor_command', val);
              }
          }
          // Animate the values
          leftDotX.value = withTiming(50 + clampedAxes[0] * 45, { duration: 0, easing: Easing.linear });
          leftDotY.value = withTiming(50 + clampedAxes[1] * 45, { duration: 0, easing: Easing.linear });
          rightDotX.value = withTiming(50 + clampedAxes[2] * 45, { duration: 0, easing: Easing.linear });
          rightDotY.value = withTiming(50 + clampedAxes[3] * 45, { duration: 0, easing: Easing.linear });
      }
    }, [names, nameColors]);

  // Controller Inputs
  useEffect(() => {
    if (firstGamepad) {
      updateJoysticks(firstGamepad);
      updateButtons(firstGamepad);
    }
  }, [firstGamepad, updateJoysticks, updateButtons]);

  // Left Dot Animation
  const leftDotStyle = useAnimatedStyle(() => ({
    left: leftDotX.value,
    top: leftDotY.value,
  }));

  // Right Dot Animation
  const rightDotStyle = useAnimatedStyle(() => ({
    left: rightDotX.value,
    top: rightDotY.value,
  }));

  // User ID Reference
  const userIDRef = useRef(userID);
  useEffect(() => {
    userIDRef.current = userID;
  }, [userID]);

  // Reload Page
  useEffect(() => {

      const reloadPageHandler = (receivedUserID: string) => {

        // Testing
        //console.log('Reload page event received from user:', receivedUserID);
        //console.log('Current user:', userIDRef.current);
        
        if (isWeb) {

          if (userIDRef.current === receivedUserID) {
            // Reload the webpage for the specified user
            window.location.reload();
          }

        } else {

          // To-Do: Reload the app on native platforms

        }
      };

      socket.on('reload_page', reloadPageHandler);

      return () => {
        socket.off('reload_page', reloadPageHandler);
        socket.disconnect();
      };

  }, []);

  return (
    <Pressable style={[styles.button, buttonStyle.button]} onPress={onPress}>
      <View style={styles.gridContainer}>
        {/* Left Grid */}
        <View style={styles.grid}>
          <View style={styles.circle} />
          <Animated.View style={[styles.dot, leftDotStyle]} />
          <View style={styles.xAxis} />
          <View style={styles.yAxis} />
        </View>
        {/* Right Grid */}
        <View style={[styles.grid, styles.rightGrid]}>
          <View style={styles.circle} />
          <Animated.View style={[styles.dot, rightDotStyle]} />
          <View style={styles.xAxis} />
          <View style={styles.yAxis} />
        </View>
      </View>
    </Pressable>
  );
};

// Controller Input Display Styles
const styles = StyleSheet.create({
  button: {
    justifyContent: 'center',
    alignItems: 'center',
  },
  gridContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: 5,
    marginRight: 20,
  },
  grid: {
    position: 'relative',
    height: isAndroid || isIOS ? 51 : 101,
    width: isAndroid || isIOS ? 51 : 101,
    borderWidth: 1,
    borderColor: 'white',
  },
  rightGrid: {
    marginLeft: -1, // Adjust this value to remove the overlapping border
  },
  circle: {
    position: 'relative',
    width: isAndroid || isIOS ? 50 : 100, // Diameter of the circle
    height: isAndroid || isIOS ? 50 : 100, // Diameter of the circle
    borderRadius: isAndroid || isIOS ? 25 : 50, // Radius of the circle
    borderWidth: 2, // Border width
    borderColor: 'white', // Border color
    backgroundColor: 'transparent', // Transparent inner color
    marginLeft: isAndroid || isIOS ? -0.75 : -0.25,
  },
  dot: {
    position: 'absolute',
    top: '50%',
    left: '50%',
    width: isAndroid || isIOS ? 7.5 : 15,
    height: isAndroid || isIOS ? 7.5 : 15,
    borderRadius: isAndroid || isIOS ? 3.75 : 7.5,
    backgroundColor: 'white',
    transform: [
        { translateX: -((isAndroid || isIOS ? 58 : 15) / 2) }, // Center horizontally
        { translateY: -((isAndroid || isIOS ? 58 : 15) / 2) }, // Center vertically
    ],
  },
  xAxis: {
    position: 'absolute',
    top: '50%',
    width: '100%',
    height: 1,
    backgroundColor: 'white',
  },
  yAxis: {
    position: 'absolute',
    left: '50%',
    height: '100%',
    width: 1,
    backgroundColor: 'white',
  },
});
  
export default Controller;