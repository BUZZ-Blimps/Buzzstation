// ========== Constants ========== //

//==== SocketIO ====//

// IO
import io from 'socket.io-client';

// Backend URL
import { url } from '../../Config/Backend_URL'

// Websocket
export const socket = io(url);

//==== Platform Type ====//

import { Platform } from 'react-native';

// IOS
export const isIOS = Platform.OS === 'ios';

// Android
export const isAndroid = Platform.OS === 'android';

// Web
export const isWeb = Platform.OS === 'web';
