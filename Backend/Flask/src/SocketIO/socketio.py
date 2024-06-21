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

# Blimp Button Colors
global blimp_button_colors
blimp_button_colors = {}

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# User Connection to Backend
@socketio.on('connect')
def handle_connect():
    # Testing
    logger.info('User Connected')

    # Get Values upon Connection
    get_redis_values()

# Changing Buttons with Two Colors
@socketio.on('toggle_button_color')
def toggle_button_color(key):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Change and Set Redis Value
    button_color = not button_color
    redis_client.set(key, int(button_color))

# Changing Buttons with 3 Colors
@socketio.on('toggle_blimp_button')
def toggle_blimp_button(val):

    # Retrieve Blimp Name and UserID
    blimp = val['blimp']
    userID = val['userID']

    global blimp_button_colors

    # Turn on Piloting for Specified User
    if blimp not in blimp_button_colors:
        
        # Only allow each user to pilot one blimp
        if userID not in blimp_button_colors.values():

            # Add Blimp Name and UserID to Dictionary
            blimp_button_colors[blimp] = userID

            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_blimp_button_color', { 'userID': userID, 'blimp': blimp})

    # Turn off Piloting (Disconnect from blimp)
    elif blimp_button_colors[blimp] == userID:

        # Remove Blimp Name and UserID from Dictionary
        del blimp_button_colors[blimp]

        # Make Blimp Button Green for all users
        socketio.emit('toggle_blimp_button_color', { 'userID': 'none', 'blimp': blimp})

    # Store Blimp Button Colors to Redis
    redis_client.set("blimp_button_colors", json.dumps(blimp_button_colors))

def get_button_color(key, default_color, nondefault_color):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Send Update to Frontend
    if (button_color == False):
        socketio.emit('update_button_color', {'blimp': 'none', 'key': key, 'color': default_color})
    else:
        socketio.emit('update_button_color', {'blimp': 'none', 'key': key, 'color': nondefault_color})

def get_blimp_names():
    # Get Current Blimp Names from Redis
    current_blimp_names = redis_client.get('current_blimp_names').decode("utf-8")

    # Update Blimp Names on Frontend
    socketio.emit('update_blimp_names', current_blimp_names.split(','))

# Get Blimp Button Colors for Each Blimp
def get_blimp_button_colors():
    # Get Current Blimp Names from Redis
    current_blimp_names = redis_client.get('current_blimp_names').decode("utf-8")

    for blimp in current_blimp_names.split(','):
        if blimp in blimp_button_colors:
            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_blimp_button_color', { 'userID': blimp_button_colors[blimp], 'blimp': blimp})
        else:
             # Make Blimp Button Green for all users
            socketio.emit('toggle_blimp_button_color', { 'userID': 'none', 'blimp': blimp})

# Get Redis Values and Sends to Frontend UI
def get_redis_values():

    # List of all Redis Values #
    get_blimp_names()

    # Blimp Button Colors
    get_blimp_button_colors()

    # Goal Color Button (Default: Orange, Nondefault: Yellow)
    get_button_color('goal_color', 'orange', 'yellow')
    
    # Enemy Color Button (Default: Blue, Nondefault: Red)
    get_button_color('enemy_color', 'blue', 'red')

    # ...

# Initialize Global Values
def init_redis_values():

    # Set to Default Value when Server Starts
    redis_client.set('goal_color', 0) # 0: Orange, 1: Yellow
    redis_client.set('enemy_color', 0) # 0: Blue, 1: Red
    redis_client.set('all_auto_state', 0) # 0: False, 1: True
    redis_client.set('current_blimp_names', "") # Empty on Start
    redis_client.set("blimp_button_colors", "{}") # Empty on Start

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
