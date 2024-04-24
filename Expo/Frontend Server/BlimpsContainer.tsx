// Button Container Component //

// React
import React, { useState, useEffect } from 'react';

// React Native
import { View, StyleSheet, Pressable, Text, Platform } from 'react-native';

// SocketIO
import { socket } from './Constants'; // Importing the SocketIO instance

// IOS
const isIOS = Platform.OS === 'ios';

// Android
const isAndroid = Platform.OS === 'android';

const BlimpsContainer: React.FC = () => {
  const [blimps, setBlimps] = useState<string[]>([]);

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

  return (
    <View style={styles.container}>
      {blimps.length > 0 ? (
        blimps.map((blimp, index) => (
          <Pressable
            key={index}
            style={styles.blimpButton}
            onPress={() => console.log(`Blimp ${blimp} pressed`)}
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
