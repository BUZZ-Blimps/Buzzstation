// ========== Constants ========== //

//==== SocketIO ====//

import io from 'socket.io-client';

// URL // To-Do: Eventually will be a Static IP where Backend Program Runs, Also add URL to a yaml file
export const url = 'http://192.168.0.202:5000'

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
