// Controller.tsx

// React
import React, { useEffect, useState, useRef, useCallback, useMemo } from 'react';

// React Native
import { View, Pressable, StyleSheet } from 'react-native';

// React Native Animations
import Animated, { Easing, useSharedValue, withTiming, useAnimatedStyle } from 'react-native-reanimated';

// Constants
import { socket, isIOS, isAndroid, isWeb } from '../Constants/Constants';

// User ID
import { getUserID } from '../Users/UserManager';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../Redux/Store';
import { setSidebarMenuState } from '../Redux/States'; 

// Gamepads
import { useGamepads } from "react-gamepads";

interface ButtonProps {
    blimpName: string; // Name of the blimp
    buttonKey: string; // Unique key for the button
    buttonColor: string; // Color of the button
    onPress: () => void; // Handle Click Function
}

const Controller: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, onPress }) => {
    const dispatch = useDispatch();
    const sidebarMenuState = useSelector((state: RootState) => state.app.sidebarMenuState);
    const { userID } = getUserID();

    // Dot Values
    const leftDotX = useSharedValue(50);
    const leftDotY = useSharedValue(50);
    const rightDotX = useSharedValue(50);
    const rightDotY = useSharedValue(50);

    // Names and Colors
    const names = useSelector((state: RootState) => state.app.names);
    const nameColors = useSelector((state: RootState) => state.app.nameColors);

    // Axes and Button States
    const prevAxesRef = useRef([0, 0, 0, 0]);
    const [buttonStates, setButtonStates] = useState<{ [key: string]: boolean }>({});
    const prevButtonStatesRef = useRef<{ [key: string]: boolean }>({});

    // GamePads
    const [gamepads, setGamepads] = useState<{ [index: number]: Gamepad }>({});
    if (isWeb) useGamepads((gamepads) => setGamepads(gamepads));
    const firstGamepad = Object.values(gamepads)[0];

    // Memoized blimp names
    const blimpNames = useMemo(() => names.filter((name) => nameColors[name] === '#006FFF'), [names, nameColors]);

    // Helper function to emit socket events
    const emitSocketEvent = (event: string, payload: Record<string, any>) => {
        if (socket) socket.emit(event, payload);
    };

    // Update Buttons
    const updateButtons = useCallback((gamepad: Gamepad) => {
        const newButtonStates = gamepad.buttons.reduce((acc, button, index) => {
            acc[`button${index}`] = button.pressed;
            return acc;
        }, {} as { [key: string]: boolean });

        const prevButtonStates = prevButtonStatesRef.current;
        let stateChanged = false;

        // Handle state changes
        for (const key in newButtonStates) {
            if (prevButtonStates[key] !== newButtonStates[key]) {
                stateChanged = true;

                if (prevButtonStates[key] && !newButtonStates[key]) {
                    const count = blimpNames.reduce((count, name) => {
                        const val = { name, button: key, userID: String(userID) };
                        emitSocketEvent('blimp_button', val);

                        if (val.button === 'button8') onPress();
                        if (val.button === 'button9') dispatch(setSidebarMenuState(!sidebarMenuState));

                        return count + 1;
                    }, 0);

                    // Handle non-blimp buttons if no blimps are connected
                    if (count === 0) {
                        const val = { button: key, userID: String(userID) };
                        if (val.button === 'button8') onPress();
                        if (val.button === 'button9') dispatch(setSidebarMenuState(!sidebarMenuState));
                        emitSocketEvent('nonblimp_button', val);
                    }
                }
            }
        }

        if (stateChanged) {
            setButtonStates((prev) => ({ ...prev, ...newButtonStates }));
            prevButtonStatesRef.current = newButtonStates;
        }
    }, [socket, blimpNames, userID, onPress, dispatch, sidebarMenuState]);

    // Update Joysticks
    const updateJoysticks = useCallback((gamepad: Gamepad) => {
        const { axes } = gamepad;
        const deadZone = 0.1;

        const deadZoneApplied = (value: number) => (Math.abs(value) < deadZone ? 0 : value);
        const clampedAxes = axes.map((axis) => parseFloat(deadZoneApplied(axis).toFixed(2)));

        const prevAxes = prevAxesRef.current;
        const axesChanged = !clampedAxes.every((value, index) => value === prevAxes[index]);

        if (axesChanged) {
            prevAxesRef.current = clampedAxes;
            blimpNames.forEach((name) => {
                const val = { name, axes: clampedAxes };
                emitSocketEvent('motor_command', val);
            });

            // Animate the values
            [leftDotX, leftDotY, rightDotX, rightDotY].forEach((dot, index) => {
                dot.value = withTiming(50 + clampedAxes[index] * 45, { duration: 0, easing: Easing.linear });
            });
        }
    }, [socket, blimpNames]);

    // Joystick and Button Inputs
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
          if (isWeb && userIDRef.current === receivedUserID) {
              window.location.reload();
          }
      };

      if (socket) {
          socket.on('reload_page', reloadPageHandler);
          return () => {
              socket.off('reload_page', reloadPageHandler);
          };
      }
    }, [socket]);

    return (
        <Pressable style={styles.button} onPress={onPress}>
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
    width: isAndroid || isIOS ? 200 : 235,
    height: isAndroid || isIOS ? 40 : 100,
    left: isAndroid || isIOS ? '35%' : '0%',
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 10,
    marginTop: 0,
    marginRight: 0,
    borderColor: 'white',
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
