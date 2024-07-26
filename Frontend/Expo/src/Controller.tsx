// Controller.tsx

// React
import React, { useEffect, useState, useRef } from 'react';

// React Native
import { View, TouchableOpacity, StyleSheet, Animated, Platform, ViewStyle, TextStyle } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Check the platform
const isWeb = Platform.OS === 'web';

// Names
import { useNames } from './Names'; // Import the useNames hook

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
    const leftDotX = useRef(new Animated.Value(50)).current;
    const leftDotY = useRef(new Animated.Value(50)).current;
    const rightDotX = useRef(new Animated.Value(50)).current;
    const rightDotY = useRef(new Animated.Value(50)).current;

    // Names
    const { names, nameColors } = useNames();

    // GamePads
    const [gamepads, setGamepads] = useState<{ [index: number]: Gamepad }>({});

    if (isWeb) {
      useGamepads((gamepads) => setGamepads(gamepads));
    }
  
    const moveDots = (gamepad: Gamepad) => {
      if (!gamepad) return;
      const { axes } = gamepad;

      let leftStickX = axes[0] || 0;
      let leftStickY = axes[1] || 0;
      let rightStickX = axes[2] || 0;
      let rightStickY = axes[3] || 0;
  
      // Apply dead zone and other processing as before
      const deadZero = 0.1;
      const deadOne = 0.01;
  
      leftStickX = Math.abs(leftStickX) < deadZero ? 0 : leftStickX;
      leftStickY = Math.abs(leftStickY) < deadZero ? 0 : leftStickY;
      rightStickX = Math.abs(rightStickX) < deadZero ? 0 : rightStickX;
      rightStickY = Math.abs(rightStickY) < deadZero ? 0 : rightStickY;
  
      leftStickX = leftStickX > 1 - deadOne ? 1 : leftStickX;
      leftStickY = leftStickY > 1 - deadOne ? 1 : leftStickY;
      rightStickX = rightStickX > 1 - deadOne ? 1 : rightStickX;
      rightStickY = rightStickY > 1 - deadOne ? 1 : rightStickY;
  
      leftStickX = leftStickX < -1 + deadOne ? -1 : leftStickX;
      leftStickY = leftStickY < -1 + deadOne ? -1 : leftStickY;
      rightStickX = rightStickX < -1 + deadOne ? -1 : rightStickX;
      rightStickY = rightStickY < -1 + deadOne ? -1 : rightStickY;
  
      leftStickX = parseFloat(leftStickX.toFixed(2));
      leftStickY = parseFloat(leftStickY.toFixed(2));
      rightStickX = parseFloat(rightStickX.toFixed(2));
      rightStickY = parseFloat(rightStickY.toFixed(2));

      const new_axes = [leftStickX, leftStickY, rightStickX, rightStickY];

      for (let index = 0; index < names.length; index++) {
        const name = names[index];
        if (nameColors[name] === '#006FFF') {
          // Testing
          //console.log(new_axes);

          let val: { name?: string; axes?: number[] } = {};
          val['name'] = name;
          val['axes'] = new_axes;
          
          // Emit to Backend
          socket.emit('motor_command', val);
        }
      }
  
      // Animate the dots
      Animated.parallel([
        Animated.timing(leftDotX, {
            toValue: 50 + leftStickX * 45,
            duration: 0, // Reduced duration
            useNativeDriver: false,
        }),
        Animated.timing(leftDotY, {
            toValue: 50 + leftStickY * 45,
            duration: 0, // Reduced duration
            useNativeDriver: false,
        }),
        Animated.timing(rightDotX, {
            toValue: 50 + rightStickX * 45,
            duration: 0, // Reduced duration
            useNativeDriver: false,
        }),
        Animated.timing(rightDotY, {
            toValue: 50 + rightStickY * 45,
            duration: 0, // Reduced duration
            useNativeDriver: false,
        })
      ]).start();
    };

    useEffect(() => {
        // Handle gamepad data and update dots
        const firstGamepad = Object.values(gamepads)[0];
        console.log(gamepads);
        moveDots(firstGamepad);
    }, [gamepads]);
  
    return (
      <TouchableOpacity style={[styles.button, buttonStyle.button]} onPress={onPress}>
        <View style={styles.gridContainer}>
          {/* Left Grid */}
          <View style={styles.grid}>
            <View style={styles.circle} />
            <Animated.View style={[styles.dot, { left: leftDotX, top: leftDotY }]} />
            <View style={styles.xAxis} />
            <View style={styles.yAxis} />
          </View>
          {/* Right Grid */}
          <View style={[styles.grid, styles.rightGrid]}>
            <View style={styles.circle} /> 
            <Animated.View style={[styles.dot, { left: rightDotX, top: rightDotY }]} />
            <View style={styles.xAxis} />
            <View style={styles.yAxis} />
          </View>
        </View>
      </TouchableOpacity>
    );
  };
  
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