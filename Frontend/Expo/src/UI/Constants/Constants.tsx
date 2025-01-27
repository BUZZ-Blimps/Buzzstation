// ========== Constants ========== //

//==== SocketIO ====//

// IO
import io from 'socket.io-client';

// Websocket
export const socket = io(process.env.EXPO_PUBLIC_BACKEND_URL);

//==== Platform Type ====//

import { Platform } from 'react-native';

// IOS
export const isIOS = Platform.OS === 'ios';

// Android
export const isAndroid = Platform.OS === 'android';

// Web
export const isWeb = Platform.OS === 'web';
