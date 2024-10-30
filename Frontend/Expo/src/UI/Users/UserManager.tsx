// UserManager.tsx

// React and React Native
import { useEffect, useState, useRef } from 'react';
import { AppState, Platform } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

// Unique User ID
import uuid from 'react-native-uuid'; 

// Async Storage on Device
import AsyncStorage from '@react-native-async-storage/async-storage';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState, AppDispatch } from '../Redux/Store';
import { setUserID } from '../Redux/States';


export const getUserID = () => {

    const dispatch: AppDispatch = useDispatch();

    // User ID
    const userID = useSelector((state: RootState) => state.app.userID);

    // Set User ID
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
                dispatch(setUserID(storedUUID));
                // setUser(storedUUID);
            }

        };

        getOrCreateUserId();

    }, [userID]);

    // User Inactivity Timer
    const userInactivityTimer = useRef<NodeJS.Timeout | null>(null);

    // Inactive Users (Web)
    useEffect(() => {

        if (isWeb) {

            const handleVisibilityChange = () => {
                const nowVisible = !document.hidden;
                
                if (nowVisible) {
                    
                    // Testing
                    // console.log('Page is visible for ${user?.ID}');
                    
                    // Clear the user inactivity timer if page becomes visible
                    if (userInactivityTimer.current) {
                        clearTimeout(userInactivityTimer.current);
                        userInactivityTimer.current = null;
                    }

                } else {
                    // Set a timer to check if the page remains hidden for more than 30 seconds
                    userInactivityTimer.current = setTimeout(() => {
                        
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

                if (userInactivityTimer.current) {
                    clearTimeout(userInactivityTimer.current);
                }
            };
        }

    }, [userID]);

    // Inactive Users (App)
    useEffect(() => {

        const handleAppStateChange = (nextAppState: string) => {

            // The app is going to the background or is inactive
            if (nextAppState === 'background' || nextAppState === 'inactive') {

              // Testing
              //console.log('App has moved to the background or inactive state');

              if (userID) {

                // Inactive User (Disconnection)
                socket.emit('inactive_user', userID);

              }

            } else if (nextAppState === 'active') {

              // Testing
              //console.log('App is in the foreground');

            }
        };

        const appStateSubscription = AppState.addEventListener('change', handleAppStateChange);

        return () => {
            appStateSubscription.remove(); // Correct way to remove the listener
        };

    }, [userID]);

    return { userID };
};
