// HandleLogoPress.ts

// React Native
import { Platform } from 'react-native';

// Expo Updates
import * as Updates from 'expo-updates';

// SocketIO
import { socket } from '../../Constants/Constants';

// Toggle Full Screen
import { toggleFullScreen } from './ToggleFullScreen';

export const handleLogoPress = (userID: string | null) => {
  
  if (Platform.OS === 'web') {

      toggleFullScreen();

  } else {

      const reloadApp = async () => {
          await Updates.reloadAsync();
      };

      // Emit inactive user event before reloading the app
      if (userID) {
          socket.emit('inactive_user', userID);
      }

      reloadApp();
  }

};