// ========== Constants ========== //

//==== SocketIO ====//

// IO
import io from 'socket.io-client';

// Websocket
export const socket = io("192.168.0.200:5000");

//==== Platform Type ====//

import { Platform } from 'react-native';

// IOS
export const isIOS = Platform.OS === 'ios';

// Android
export const isAndroid = Platform.OS === 'android';

// Web
export const isWeb = Platform.OS === 'web';
