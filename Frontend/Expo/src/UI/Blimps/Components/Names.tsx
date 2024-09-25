// Names.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

// User ID's
import { getUserID } from '../../Users/UserManager';
import { Float } from 'react-native/Libraries/Types/CodegenTypes';

export const useNames = () => {

  // User ID
  const { userID } = getUserID();

  // Names
  const [names, setNames] = useState<string[]>([]);

  // Name Button Colors
  const [nameColors, setNameColors] = useState<{ [key: string]: string }>({});

  // Name Last Hearbeats
  const [nameLastHeartbeat, setNameLastHeartbeats] = useState<{ [key: string]: Float }>({});

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

  }, []);

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

  // Update Name Last Heartbeats
  useEffect(() => {

    const handleNameLastHeartbeat = (val: { name: string; time_since_last_heartbeat: Float }) => {
      const { name: receivedName, time_since_last_heartbeat: receivedLastHeartbeat } = val;

      // Testing
      // console.log(receivedName);
      // console.log(receivedLastHeartbeat);

      setNameLastHeartbeats((prevLastHeartbeats) => ({
        ...prevLastHeartbeats,
        [receivedName]: receivedLastHeartbeat,
      }));
    };

    socket.on('name_time_since_last_heartbeat', handleNameLastHeartbeat);

    return () => {
      socket.off('name_time_since_last_heartbeat', handleNameLastHeartbeat);
    };

  }, [nameLastHeartbeat]);

  // Name Button Click
  const handleNameClick = (name: string) => {
    if (name !== 'none') {
      if (userID) {

        socket.emit('toggle_name_button', { userID: userID, name: name });

        // Testing
        //console.log('Blimp ' + name + ' pressed by ' + userID);

      }
    }
  }

  // Calculate normalized heartbeat
  const getNormalizedHeartbeat = (name: string): number => {
    const maxHeartbeat = 5.0;
    const heartbeat = nameLastHeartbeat[name] || 0;
    // console.log('Name: ' + name);
    // console.log('Heartbeat: ' + heartbeat);
    // console.log('Normalized: ' + Math.round((Math.min(heartbeat / maxHeartbeat, 1) + Number.EPSILON) * 100) / 100);
    return Math.round((Math.min(heartbeat / maxHeartbeat, 1) + Number.EPSILON) * 100) / 100; // Rounded to 2 decimal places
  };

  // Name Button Style
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
      userSelect: 'none',
    },
  });

  return {
    names,
    nameColors,
    nameButtonStyle,
    getNormalizedHeartbeat,
    handleNameClick,
    userID,
  };

};
