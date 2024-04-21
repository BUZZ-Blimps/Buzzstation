# ========== SocketIO ========== #

"""
Description:

Main SocketIO file where data is sent and recieved by the Backend Server.
Data is sent to the Frontend Server to update every device's UI.

To-Do:

- Separate this file into more files (senders.py, recievers.py, ...)

"""

# Imports
from Packages.packages import *

# User Connection to Backend
@socketio.on('connect')
def handle_connect():

    # Get Values upon Connection
    get_redis_values()

# Sender
def get_button_color(key, default_color, nondefault_color):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Send Update to Frontend
    if (button_color == False):
        socketio.emit('update_button_color', {'key': key, 'color': default_color})
    else:
        socketio.emit('update_button_color', {'key': key, 'color': nondefault_color})

# Reciever
@socketio.on('toggle_button_color')
def toggle_button_color(key):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Change and Set Redis Value
    button_color = not button_color
    redis_client.set(key, int(button_color))

# Get Redis Values and Sends to Frontend UI
def get_redis_values():

    # List of all Redis Values

    # Goal Color Button (Default: Orange, Nondefault: Yellow)
    get_button_color('goal_color', 'orange', 'yellow')
    get_button_color('enemy_color', 'blue', 'red')
    # ...

# Initialize Global Values
def init_redis_values():

    # Set to Default Value when Server Starts
    redis_client.set('goal_color', 0) # 0: Orange, 1: Yellow
    redis_client.set('enemy_color', 0) # 0: Blue, 1: Red
    redis_client.set('all_auto_state', 0) # 0: False, 1: True

# Start Backend Server
def start_backend_server(host, port):

    # Start App at the specified IP and port
    try:
        socketio.run(app, host=host, port=port)
    except:
        try:
            socketio.run(app, allow_unsafe_werkzeug=True, host=host, port=port)
        except:
            pass
