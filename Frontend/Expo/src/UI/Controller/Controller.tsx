// Controller.tsx

// React
import React, { useEffect } from 'react';

// React Native
import { View, Pressable, StyleSheet } from 'react-native';

// React Native Animations
import Animated, { Easing, useSharedValue, withTiming, useAnimatedStyle } from 'react-native-reanimated';

// Constants
import { socket, isIOS, isAndroid } from '../Constants/Constants';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../Redux/Store';
import { setSidebarMenuState } from '../Redux/States'; 

// Controller input is read by the Backend over ROS (joy_node), this component only displays it

interface ButtonProps {
    blimpName: string; // Name of the blimp
    buttonKey: string; // Unique key for the button
    buttonColor: string; // Color of the button
    onPress: () => void; // Handle Click Function
}

const Controller: React.FC<ButtonProps> = ({ blimpName, buttonKey, buttonColor, onPress }) => {
    const dispatch = useDispatch();
    const sidebarMenuState = useSelector((state: RootState) => state.app.sidebarMenuState);

    // Dot Values
    const leftDotX = useSharedValue(50);
    const leftDotY = useSharedValue(50);
    const rightDotX = useSharedValue(50);
    const rightDotY = useSharedValue(50);

    // Motor Commands from Backend ([Left X, Left Y, Right X, Right Y], Right and Up are positive)
    useEffect(() => {
        const motorCommandsHandler = (axes: number[]) => {
            const [leftX, leftY, rightX, rightY] = axes;
            leftDotX.value = withTiming(50 + leftX * 45, { duration: 0, easing: Easing.linear });
            leftDotY.value = withTiming(50 - leftY * 45, { duration: 0, easing: Easing.linear });
            rightDotX.value = withTiming(50 + rightX * 45, { duration: 0, easing: Easing.linear });
            rightDotY.value = withTiming(50 - rightY * 45, { duration: 0, easing: Easing.linear });
        };

        if (socket) {
            socket.on('motor_commands', motorCommandsHandler);
            return () => {
                socket.off('motor_commands', motorCommandsHandler);
            };
        }
    }, [socket]);

    // Controller View and Menu Buttons
    useEffect(() => {
        const toggleOverlayImageHandler = () => onPress();
        const toggleSidebarMenuHandler = () => dispatch(setSidebarMenuState(!sidebarMenuState));

        if (socket) {
            socket.on('toggle_overlay_image', toggleOverlayImageHandler);
            socket.on('toggle_sidebar_menu', toggleSidebarMenuHandler);
            return () => {
                socket.off('toggle_overlay_image', toggleOverlayImageHandler);
                socket.off('toggle_sidebar_menu', toggleSidebarMenuHandler);
            };
        }
    }, [socket, onPress, dispatch, sidebarMenuState]);

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
