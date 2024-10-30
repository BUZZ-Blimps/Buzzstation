# ========== SocketIO ========== #

"""
Description:

Main SocketIO file where data is sent and recieved by the Backend Server.
Data is sent to the Frontend Server to update every device's UI.

SocketIO Event Types:
- User
- Button
- Controller

"""

# Imports
from Packages.packages import socketio, app

# Blimp Name Button Colors
global name_button_colors
name_button_colors = {}

# User Events
import SocketIO.Receivers.users

# Button Events
import SocketIO.Receivers.buttons

# Controller Events
import SocketIO.Receivers.controller

# Start Backend Server
def start_backend_server(host, port):

    # Start App at the specified IP and port
    try:
        socketio.run(app, host=host, port=port, use_reloader=False)
    except:
        try:
            socketio.run(app, allow_unsafe_werkzeug=True, host=host, port=port, use_reloader=False)
        except:
            pass
