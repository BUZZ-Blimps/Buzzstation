// Names.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet } from 'react-native';
import ReactHlsPlayer from 'react-hls-player';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../../Constants/Constants';

// User ID
import { getUserID } from '../../Users/UserManager';
import { Float } from 'react-native/Libraries/Types/CodegenTypes';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState, AppDispatch } from '../../Redux/Store';
import { setNames, setNameColors, setNameLastHeartbeats } from '../../Redux/States';

const stringMap = {
  "BurnCream" : "1. BurnCream",
  "SillyAh" : "2. SillyAh",
  "Turbo" : "3. Turbo",
  "GameChamber" : "4. GameChamber",
  "GravyLongWay" : "5. GravyLongWay",
  "SuperBeef" : "6. SuperBeef"
};

export const useNames = () => {

  // Redux Dispatch
  const dispatch: AppDispatch = useDispatch();

  // User ID
  const { userID } = getUserID();

  // Names
  const names = useSelector((state: RootState) => state.app.names);

  // Name Button Colors
  const nameColors = useSelector((state: RootState) => state.app.nameColors);

  // Name Last Hearbeats
  const nameLastHeartbeats = useSelector((state: RootState) => state.app.nameLastHeartbeats);

  // Update Names
  useEffect(() => {

    // Define the event handler for 'update_names'
    const handleUpdateNames = (data: string[]) => {
      if (data[0] !== '') {

        const modified_data = data.map((item, index) => stringMap[item]);

        dispatch(setNames(modified_data)); // Update state with new names list
      } else {
        dispatch(setNames([])); // If no data, set to empty array
      }
    };

    if (socket) {
      // Listen for 'update_names' events
      socket.on('update_names', handleUpdateNames);

      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_names', handleUpdateNames);
      };
    }

  }, [socket, names, dispatch]);

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

      dispatch(setNameColors({ [receivedName]: newColor }));
    };

    if (socket) {
      socket.on('toggle_name_button_color', handleToggleNameButtonColor);

      return () => {
        socket.off('toggle_name_button_color', handleToggleNameButtonColor);
      };
    }

  }, [socket, userID, nameColors, dispatch]);

  // Update Name Last Heartbeats
  useEffect(() => {

    const handleNameLastHeartbeat = (val: { name: string; time_since_last_heartbeat: Float }) => {
      const { name: receivedName, time_since_last_heartbeat: receivedLastHeartbeat } = val;

      dispatch(setNameLastHeartbeats({ [receivedName]: receivedLastHeartbeat }));
    };

    if (socket) {
      socket.on('name_time_since_last_heartbeat', handleNameLastHeartbeat);

      return () => {
        socket.off('name_time_since_last_heartbeat', handleNameLastHeartbeat);
      };
    }

  }, [socket, nameLastHeartbeats, dispatch]);

  // Name Button Click
  const handleNameClick = (name: string) => {
    if (name !== 'none' && userID && socket) {

      socket.emit('toggle_name_button', { userID: userID, name: name });
    
      // // Testing
      // console.log('Blimp ' + name + ' pressed by ' + userID);
    }
  }

  // Calculate normalized heartbeat
  const getNormalizedHeartbeat = (name: string): number => {
    const maxHeartbeat = 5.0;
    const heartbeat = nameLastHeartbeats[name] || 0;
    // console.log('Name: ' + name);
    // console.log('Heartbeat: ' + heartbeat);
    // console.log('Normalized: ' + Math.round((Math.min(heartbeat / maxHeartbeat, 1) + Number.EPSILON) * 100) / 100);
    const normalizedHeartbeat = Math.round((Math.min(heartbeat / maxHeartbeat, 1) + Number.EPSILON) * 100) / 100; // Rounded to 2 decimal places

    return normalizedHeartbeat
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
