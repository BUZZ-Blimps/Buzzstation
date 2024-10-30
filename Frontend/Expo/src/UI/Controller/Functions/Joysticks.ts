// Joysticks.ts
import { useCallback } from 'react';
import { SharedValue, withTiming, Easing } from 'react-native-reanimated';
import { socket } from '../../Constants/Constants';

export const useJoysticks = (
    firstGamepad: Gamepad | undefined,
    blimpNames: string[],
    leftDotX: SharedValue<number>,
    leftDotY: SharedValue<number>,
    rightDotX: SharedValue<number>,
    rightDotY: SharedValue<number>
) => {
    return useCallback(() => {
        if (firstGamepad) {
            const { axes } = firstGamepad;
            const [leftStickX, leftStickY, rightStickX, rightStickY] = axes;

            const deadZone = 0.1;
            const deadZoneApplied = (value: number) => (Math.abs(value) < deadZone ? 0 : value);
            const clampedAxes = [
                parseFloat(deadZoneApplied(leftStickX).toFixed(2)),
                parseFloat(deadZoneApplied(leftStickY).toFixed(2)),
                parseFloat(deadZoneApplied(rightStickX).toFixed(2)),
                parseFloat(deadZoneApplied(rightStickY).toFixed(2)),
            ];

            blimpNames.forEach((name) => {
                const val = { name, axes: clampedAxes };
                if (socket) {
                    socket.emit('motor_command', val);
                }
            });

            leftDotX.value = withTiming(50 + clampedAxes[0] * 45, { duration: 0, easing: Easing.linear });
            leftDotY.value = withTiming(50 + clampedAxes[1] * 45, { duration: 0, easing: Easing.linear });
            rightDotX.value = withTiming(50 + clampedAxes[2] * 45, { duration: 0, easing: Easing.linear });
            rightDotY.value = withTiming(50 + clampedAxes[3] * 45, { duration: 0, easing: Easing.linear });
        }
    }, [firstGamepad, blimpNames, leftDotX, leftDotY, rightDotX, rightDotY]);
};
