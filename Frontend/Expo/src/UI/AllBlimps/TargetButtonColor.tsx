// TargetButtonColor.tsx

// React and React Native
import { useState, useEffect } from 'react';
import { StyleSheet, Platform } from 'react-native';

// Constants
import { socket, isIOS, isAndroid, isWeb} from '../Constants/Constants';

// Redux
import { useDispatch, useSelector } from 'react-redux';
import { RootState, AppDispatch } from '../Redux/Store';
import { setGoalColor, setEnemyColor } from '../Redux/States';

interface Button {
  handleClick: (buttonKey: string) => void;
}

export const useTargetButtonColor = (defaultColor: string, buttonKey: string): Button => {

  // Redux Dispatch
  const dispatch: AppDispatch = useDispatch();

  // Update Button Color
  useEffect(() => {

    // Event handler for 'update_button_color'
    const handleUpdateButtonColor = (val: { [key: string]: string }) => {
      const receivedName: string = val['name'];
      const receivedButtonKey: string = val['key'];
      const receivedButtonColor: string = val['color'];

      let newColor = defaultColor; // Default color 
      if (receivedButtonColor === 'orange') {
        newColor = '#FF5D01';
      } else if (receivedButtonColor === 'yellow') {
        newColor = '#DADE00';
      } else if (receivedButtonColor === 'blue') {
        newColor = '#0044A6';
      } else if (receivedButtonColor === 'red') {
        newColor = '#D20F18';
      }

      if (receivedName === 'none') {
        if (receivedButtonKey === buttonKey) {
          if (buttonKey === 'goal_color') {
            dispatch(setGoalColor(newColor));
          }
          else if (buttonKey === 'enemy_color') {
            dispatch(setEnemyColor(newColor));
          }
          // Testing
          //console.log(buttonKey + " changed to " + receivedButtonColor);
        }
      }
    };

    if (socket) {
      // Listen for 'update_button_color' events
      socket.on('update_button_color', handleUpdateButtonColor);

      // Cleanup to remove the listener when the component is unmounted
      return () => {
        socket.off('update_button_color', handleUpdateButtonColor);
      };
    }

  }, [socket, buttonKey]);

  // Target Button Click
  const handleClick = (buttonKey: string) => {

    if (buttonKey !== 'None') {

      if (socket) {
        socket.emit('toggle_all_blimps_button_color', buttonKey);
      }

      // Testing
      //console.log(buttonKey + " button clicked!");

    }

  };

  return {
    handleClick,
  };

};
