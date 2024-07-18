// React
import React, { useEffect, useRef } from 'react';

// React Native
import { View, TouchableOpacity, StyleSheet, Animated, Platform, ViewStyle, TextStyle } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

interface ButtonStyle {
    button: ViewStyle; // Style for the button container
    buttonText: TextStyle; // Style for the text inside the button
}

interface ButtonProps {
    blimpName: string; // Name of the blimp
    buttonKey: string; // Unique key for the button
    buttonColor: string; // Color of the button
    buttonStyle: ButtonStyle; // Text Style for the button
    onPress: () => void; // Handle Click Function
}

const Controller: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, buttonStyle, onPress }) => {
    const leftDotX = useRef(new Animated.Value(50)).current;
    const leftDotY = useRef(new Animated.Value(50)).current;
    const rightDotX = useRef(new Animated.Value(50)).current;
    const rightDotY = useRef(new Animated.Value(50)).current;
  
    const moveDots = (controller_cmd: Float64Array) => {
      let leftStickX = controller_cmd[0];
      let leftStickY = controller_cmd[1];
      let rightStickX = controller_cmd[2];
      let rightStickY = controller_cmd[3];
  
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
  
      rightStickX *= -1;
  
      // Animate the dots
      Animated.timing(leftDotX, {
        toValue: 50 + leftStickX * 45,
        duration: 100,
        useNativeDriver: false,
      }).start();
      Animated.timing(leftDotY, {
        toValue: 50 + leftStickY * -45,
        duration: 100,
        useNativeDriver: false,
      }).start();
      Animated.timing(rightDotX, {
        toValue: 50 + rightStickX * 45,
        duration: 100,
        useNativeDriver: false,
      }).start();
      Animated.timing(rightDotY, {
        toValue: 50 + rightStickY * -45,
        duration: 100,
        useNativeDriver: false,
      }).start();
    };
  
    useEffect(() => {
      const handleMotorCommands = (controller_cmd: Float64Array) => {
        // Testing
        console.log(controller_cmd);
  
        moveDots(controller_cmd);
      };
  
      socket.on('motor_commands', handleMotorCommands);
  
      return () => {
        socket.off('motor_commands', handleMotorCommands);
      };
    }, []);
  
    return (
      <TouchableOpacity style={[styles.button, buttonStyle.button]} onPress={onPress}>
        <View style={styles.gridContainer}>
          {/* Left Grid */}
          <View style={styles.grid}>
            <Animated.View style={[styles.dot, { left: leftDotX, top: leftDotY }]} />
            <View style={styles.xAxis} />
            <View style={styles.yAxis} />
          </View>
          {/* Right Grid */}
          <View style={[styles.grid, styles.rightGrid]}>
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
    },
    grid: {
      position: 'relative',
      height: isAndroid || isIOS ? 50 : 100,
      width: isAndroid || isIOS ? 62.5 : 125,
      borderWidth: 1,
      borderColor: 'white',
    },
    rightGrid: {
      marginLeft: -1, // Adjust this value to remove the overlapping border
    },
    dot: {
      position: 'absolute',
      top: '50%',
      left: '50%',
      width: isAndroid || isIOS ? 7.5 : 15,
      height: isAndroid || isIOS ? 7.5 : 15,
      borderRadius: isAndroid || isIOS ? 3.75 : 7.5,
      backgroundColor: 'white',
      transform: isAndroid || isIOS ? [{ translateX: -125/4+15/2+0.5 }, { translateY: -100/4-7.5/2 }] : [{ translateX: 7.5/2+0.5/2 }, { translateY: -15/2+0.5/2 }],
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
