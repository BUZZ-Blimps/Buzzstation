// Names.tsx

// React and React Native
import React, { useState, useEffect, useRef } from 'react';
import { View, StyleSheet, Platform } from 'react-native';

// SocketIO
import { socket } from './Constants';

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

// Web
const isWeb = Platform.OS === 'web';

// Async Storage on Device
import AsyncStorage from '@react-native-async-storage/async-storage';

// Unique User ID
import uuid from 'react-native-uuid';

export const useNames = () => {
    // Store User ID in Async Storage
    const [userID, setUserId] = useState<string | null>(null);
  
    // Store Blimp Names
    const [names, setNames] = useState<string[]>([]);
  
    // Store Blimp Name Button Colors
    const [nameColors, setNameColors] = useState<{ [key: string]: string }>({});

    // Handle Inactive Users
    if (isWeb) {
      const [isVisible, setIsVisible] = useState<boolean>(!document.hidden);
      const hiddenTimeRef = useRef<NodeJS.Timeout | null>(null);

      const handleVisibilityChange = () => {
        const nowVisible = !document.hidden;
        setIsVisible(nowVisible);
    
        if (nowVisible) {

          // Testing
          //console.log('Page is visible');

          // Clear the hidden timer if page becomes visible
          if (hiddenTimeRef.current) {
            clearTimeout(hiddenTimeRef.current);
            hiddenTimeRef.current = null;
          }

        } else {
          // Set a timer to check if the page remains hidden for more than 30 seconds
          hiddenTimeRef.current = setTimeout(() => {

            // Testing
            //console.log('Page has been hidden for more than 30 seconds');
            //console.log(userID);

            // Notify backend that user has been inactive for more than 30 seconds
            socket.emit('inactive_user', userID);

          }, 30000);
        }
      };

      const handleBeforeUnload = () => {

        // Emit Inactive User
        socket.emit('inactive_user', userID);

      };
  
      useEffect(() => {
        if (isWeb) {

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
      }, [userID, nameColors]);
    }
  
    // Update Names
    useEffect(() => {
      // Define the event handler for 'update_names'
      const handleUpdateNames = (data: string[]) => {
        if (data[0] !== '') {
          setNames(data); // Update state with new names list
        } else {
          setNames([]); // If no data, set to empty array
        }
      };
  
      // Listen for 'update_names' events
      socket.on('update_names', handleUpdateNames);
  
      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_names', handleUpdateNames);
      };
    }, []); // Empty dependency array to ensure the effect runs only once when the component mounts
  
    // Update Name Color
    useEffect(() => {
      const handleToggleNameButtonColor = (val: { userID: string; name: string; color: string }) => {
        const { userID: receivedUserID, name: receivedName } = val;
  
        let newColor = '#E11C1C'; // Default color for other users // red
        if (receivedUserID === userID) {
          newColor = '#006FFF'; // Color for the current user // blue
        } else if (receivedUserID === 'none') {
          newColor = 'green'; // Color for all users if userID is 'none' // green
        }

        // Testing
        //console.log(newColor);
  
        setNameColors((prevColors) => ({
          ...prevColors,
          [receivedName]: newColor,
        }));
      };
  
      socket.on('toggle_name_button_color', handleToggleNameButtonColor);
  
      return () => {
        socket.off('toggle_name_button_color', handleToggleNameButtonColor);
      };
    }, [userID]);
  
    // Get and Store User ID's
    useEffect(() => {
      const getOrCreateUUID = async () => {
        let storedUUID = await AsyncStorage.getItem('userUUID');
        if (!storedUUID) {
          // If no UUID is found, create a new one and store it
          const newUUID = String(uuid.v4());
          await AsyncStorage.setItem('userUUID', newUUID);
          storedUUID = newUUID;
        }
        setUserId(storedUUID); // Set the UUID in state
      };
  
      getOrCreateUUID(); // Run on component mount
    }, []);
  
    // Handle Name Button Click
    const handleNameClick = (name: string) => {
      if (name !== 'none') {
        if (userID) {
          socket.emit('toggle_name_button', { userID: userID, name: name });
          // Testing
          //console.log(userID);
          // Testing
          //console.log(`Blimp ${name} pressed`);
        }
      }
    };

    const nameButtonStyle = StyleSheet.create({
      button: {
        width: 110,
        height: 40,
        padding: isAndroid || isIOS ? 9 : 7, // Center Text Vertically
        borderRadius: 5,
        borderWidth: 2,
        borderColor: 'black',
        marginHorizontal: 5, // Space between buttons
        justifyContent: 'center',
        alignItems: 'center',
      },
      buttonText: {
        fontWeight: 'bold',
        color: 'white', // Text color
        fontSize: 14, // Text size
        textAlign: 'center', // Center the text
        textShadowColor: 'black', // Outline color
        textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
        textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
      },
    });
  
    return {
      names,
      nameColors,
      nameButtonStyle,
      handleNameClick,
      userID,
    };
};
