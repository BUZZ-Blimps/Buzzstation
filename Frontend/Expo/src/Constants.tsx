// ========== Constants ========== //

// SocketIO
import io from 'socket.io-client';

// URL
// To-Do: Eventually will be a Static IP
export const url = 'http://192.168.7.212:5000'

// Websocket
export const socket = io(url);
