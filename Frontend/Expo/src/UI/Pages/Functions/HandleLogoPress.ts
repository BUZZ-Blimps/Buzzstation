// HandleLogoPress.ts

// React Native
import { Platform } from 'react-native';

// Expo Updates
import * as Updates from 'expo-updates';

// SocketIO
import { socket } from '../../Constants/Constants';

// User ID
import { getUserID } from '../../Users/UserManager';

// Toggle Full Screen
import { toggleFullScreen } from './ToggleFullScreen';

export function handleLogoPress() {
    
    if (Platform.OS === 'web') {
      toggleFullScreen();
    } else {
      const reloadApp = async () => {
        await Updates.reloadAsync();
      };
  
      reloadApp();

      const { userID } = getUserID();
  
      if (userID) {
        socket.emit('inactive_user', userID); // Emit inactive user event
      }
    }
}
