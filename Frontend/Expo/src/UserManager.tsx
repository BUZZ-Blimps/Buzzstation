// UserManager.tsx

// React and React Native
import { useEffect, useState, useRef } from 'react';
import { Platform } from 'react-native';

// SocketIO
import { socket } from './Constants';

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Web
const isWeb = Platform.OS === 'web';

// Unique User ID
import uuid from 'react-native-uuid'; 

// Async Storage on Device
import AsyncStorage from '@react-native-async-storage/async-storage';

export const getUserID = () => {

    // Set User ID
    const [userID, setUser] = useState<String | null>(null);
    useEffect(() => {
        const getOrCreateUserId = async () => {
        let storedUUID = await AsyncStorage.getItem('userUUID');
        if (!storedUUID) {
            // If no UUID is found, create a new one and store it
            const newUUID = String(uuid.v4());
            await AsyncStorage.setItem('userUUID', newUUID);
            storedUUID = newUUID;
        }
        // Add User
        if (storedUUID !== null) {
            setUser(storedUUID);
        }
        };

        getOrCreateUserId();
    }, []);

    // Inactive Users
    const hiddenTimeRef = useRef<NodeJS.Timeout | null>(null);
    useEffect(() => {
        if (isWeb) {

            const handleVisibilityChange = () => {
                const nowVisible = !document.hidden;
                
                if (nowVisible) {
                    
                    // Testing
                    // console.log('Page is visible for ${user?.ID}');
                    
                    // Clear the hidden timer if page becomes visible
                    if (hiddenTimeRef.current) {
                        clearTimeout(hiddenTimeRef.current);
                        hiddenTimeRef.current = null;
                    }
                } else {
                    // Set a timer to check if the page remains hidden for more than 30 seconds
                    hiddenTimeRef.current = setTimeout(() => {
                        
                        // Testing
                        // console.log('Page has been hidden for more than 30 seconds for ${user?.ID}');
                        
                        if (userID) {

                            // Inactive User for more than 30 Seconds
                            socket.emit('inactive_user', userID);

                        }

                    }, 30000);
                }
            };

            const handleBeforeUnload = () => {

                // Testing
                // console.log('Page is unloading for ${user?.ID}');

                if (userID) {

                    // Inactive User (Reload or Disconnection)
                    socket.emit('inactive_user', userID);

                }
            };

            document.addEventListener('visibilitychange', handleVisibilityChange);
            window.addEventListener('beforeunload', handleBeforeUnload);

            return () => {
                document.removeEventListener('visibilitychange', handleVisibilityChange);
                window.removeEventListener('beforeunload', handleBeforeUnload);

                if (hiddenTimeRef.current) {
                    clearTimeout(hiddenTimeRef.current);
                }
            };
        }
    }, [userID]);

    return { userID };
};