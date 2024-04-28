// Button Container Component //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { View, StyleSheet, Pressable, Text, Platform } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// Async Storage on Device
import AsyncStorage from '@react-native-async-storage/async-storage';

// Unique User ID
import uuid from 'react-native-uuid';

// Generate a UUID to assign a unique ID
// const userID = uuid.v4();

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

const BlimpsContainer: React.FC = () => {
  // Store Blimp Names
  const [blimps, setBlimps] = useState<string[]>([]);

  // Store User ID in Async Storage
  const [userID, setUserId] = useState<string | null>(null);

  // Store Blimp Button Colors
  const [blimpColors, setBlimpColors] = useState<{ [key: string]: string }>({});

  useEffect(() => {
    // Define the event handler for 'update_blimps'
    const handleUpdateBlimpNames = (data: string[]) => {
      if (data[0] !== '') {
        setBlimps(data); // Update state with new blimps list
      } else {
        setBlimps([]); // If no data, set to empty array
      }
    };

    // Listen for 'update_blimps' events
    socket.on('update_blimp_names', handleUpdateBlimpNames);

    // Cleanup to remove the listener when the component is unmounted
    return () => {
      socket.off('update_blimp_names', handleUpdateBlimpNames);
    };
  }, []); // Empty dependency array to ensure the effect runs only once when the component mounts

  useEffect(() => {
    const handleToggleBlimpColor = (val: { userID: string; blimp: string; color: string }) => {
      const { userID: receivedUserID, blimp: receivedBlimp } = val;

      let newColor = 'red'; // Default color for other users
      if (receivedUserID === userID) {
        newColor = 'blue'; // Color for the current user
      } else if (receivedUserID === 'none') {
        newColor = 'green'; // Color for all users if userID is 'none'
      }

      setBlimpColors((prevColors) => ({
        ...prevColors,
        [receivedBlimp]: newColor,
      }));
    };

    socket.on('toggle_blimp_button_color', handleToggleBlimpColor);

    return () => {
      socket.off('toggle_blimp_button_color', handleToggleBlimpColor);
    };
  }, [userID]);

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

  // Handle Blimp NameButton Click
  const handleClick = (blimp: string) => {
    if (blimp !== 'none') {
      if (userID) {
        socket.emit('toggle_blimp_button', { userID: userID, blimp: blimp });
        // Testing
        console.log(userID);
        console.log(`Blimp ${blimp} pressed`);
      }
    }
};

  return (
    <View style={styles.container}>
      {blimps.length > 0 ? (
        blimps.map((blimp, index) => (
          <Pressable
            key={index}
            style={[styles.blimpButton, { backgroundColor: blimpColors[blimp] || 'green' }]}
            onPress={() => handleClick(blimp)}
            android_disableSound={true}
            android_ripple={{ color: 'transparent' }}
            role='button'
            accessible={true}
          >
            <Text style={styles.blimpNameText}>{blimp}</Text>
          </Pressable>
        ))
      ) : (
        // When no blimps are available, render nothing
        <View />
      )}
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center',
    //backgroundColor: 'black', // Use for testing of Containers
  },
  blimpButton: {
    width: 110,
    height: 40,
    backgroundColor: 'green',
    padding: isAndroid || isIOS ? 9 : 7, // Center Text Vertically
    borderRadius: 5,
    borderWidth: 2,
    borderColor: 'black',
    marginVertical: 5,
  },
  blimpNameText: {
    fontWeight: 'bold',
    color: 'white', // Text color
    fontSize: 14, // Text size
    textAlign: 'center', // Center the text
    textShadowColor: 'black', // Outline color
    textShadowOffset: { width: 1, height: 1 }, // Direction of the shadow
    textShadowRadius: isAndroid || isIOS ? 0.1 : 1, // Spread of the shadow
  }
});

export default BlimpsContainer;
