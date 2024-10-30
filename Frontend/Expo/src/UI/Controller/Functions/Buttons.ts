// Buttons.ts
import { useCallback } from 'react';
import { socket } from '../../Constants/Constants';
import { setSidebarMenuState } from '../../Redux/States';
import { useDispatch } from 'react-redux';

interface ButtonProps {
    userID: string | null;
    blimpNames: string[];
    onPress: () => void;
    sidebarMenuState: boolean;
}

export const useButtons = (
    firstGamepad: Gamepad | undefined,
    { blimpNames, userID, onPress, sidebarMenuState }: ButtonProps
) => {
    const dispatch = useDispatch();

    return useCallback(() => {
        if (firstGamepad) {
            const newButtonStates = firstGamepad.buttons.reduce((acc, button, index) => {
                acc[`button${index}`] = button.pressed;
                return acc;
            }, {} as { [key: string]: boolean });

            // Handle button states here
            for (const key of Object.keys(newButtonStates)) {
                if (newButtonStates[key]) {
                    if (socket) {
                        socket.emit('blimp_button', { name: blimpNames[0], button: key, userID: String(userID) });
                    }
                    if (key === 'button8') {
                        onPress();
                    }
                    if (key === 'button9') {
                        dispatch(setSidebarMenuState(!sidebarMenuState));
                    }
                }
            }
        }
    }, [firstGamepad, blimpNames, userID, onPress, dispatch, sidebarMenuState]);
};