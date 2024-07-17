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
from ROS.Blimps_Loop.Blimps.blimp_names import blimp_names_order

# Blimp Name Button Colors
global name_button_colors
name_button_colors = {}

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

# Changing All Blimps Buttons with Two Colors
@socketio.on('toggle_all_blimps_button_color')
def toggle_all_blimps_button_color(key):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Change and Set Redis Value
    button_color = not button_color
    redis_client.set(key, int(button_color))

# Changing Blimp Buttons with Two Colors
@socketio.on('toggle_blimp_button_color')
def toggle_blimp_button_color(val):
    from ROS.ros import basestation_node

    # Retrieve Blimp Name and Key
    name = val['name']
    key = val['key']

    if hasattr(basestation_node.current_blimps[name], key):

        # Change the Value of the Key for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], key, not getattr(basestation_node.current_blimps[name], key))
        
# Set Blimp Button to specific value
@socketio.on('set_blimp_button_value')
def set_blimp_button_value(val):
    from ROS.ros import basestation_node

    # Retrieve Blimp Name, Key, and Value
    name = val['name']
    key = val['key']
    value = bool(int(val['value']))

    if hasattr(basestation_node.current_blimps[name], key):

        # Change the Value of the Key for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], key, value)

# Changing Buttons with 3 Colors
@socketio.on('toggle_name_button')
def toggle_name_button(val):

    # Retrieve Blimp Name and UserID
    name = val['name']
    userID = val['userID']

    global name_button_colors

    # Turn on Piloting for Specified User
    if name not in name_button_colors:
        
        # Only allow each user to pilot one blimp
        if userID not in name_button_colors.values():

            # Add Blimp Name and UserID to Dictionary
            name_button_colors[name] = userID

            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_name_button_color', { 'userID': userID, 'name': name})

    # Turn off Piloting (Disconnect from blimp)
    elif name_button_colors[name] == userID:

        # Remove Blimp Name and UserID from Dictionary
        del name_button_colors[name]

        # Make Blimp Name Button Green for all users
        socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

    # Store Blimp Button Colors to Redis
    redis_client.set('name_button_colors', json.dumps(name_button_colors))

# Get a Color for any Button
def get_button_color(name, key, default_color, nondefault_color):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Send Update to Frontend
    if (button_color == False):
        socketio.emit('update_button_color', {'name': name, 'key': key, 'color': default_color})
    else:
        socketio.emit('update_button_color', {'name': name, 'key': key, 'color': nondefault_color})

# Get Current Blimp Names
def get_names():
    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    # Update Blimp Names on Frontend
    socketio.emit('update_names', current_names.split(','))

# Get Blimp Button Colors for Each Blimp
def get_name_button_colors():
    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if name in name_button_colors:
            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_name_button_color', { 'userID': name_button_colors[name], 'name': name})
        else:
             # Make Blimp Button Green for all users
            socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

# Get Mode for Each Blimp
def get_mode_button_colors():

    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if redis_client.hget(str('blimp:' + name), 'mode') is not None:
            current_component_color = redis_client.hget(str('blimp:' + name), 'mode').decode('utf-8')
            # Send Update to Frontend
            if str(current_component_color) == 'True':
                socketio.emit('update_button_color', {'name': name, 'key': 'mode', 'color': 'green'})
            else:
                socketio.emit('update_button_color', {'name': name, 'key': 'mode', 'color': 'red'})

# Get Redis Values and Sends to Frontend UI
def get_redis_values():

    # List of all Redis Values #

    # Names
    get_names()

    # Name Button Colors
    get_name_button_colors()

    # Mode Button Colors
    get_mode_button_colors()

    # Goal Color Button (Default: Orange, Nondefault: Yellow)
    get_button_color('none', 'goal_color', 'orange', 'yellow')
    
    # Enemy Color Button (Default: Blue, Nondefault: Red)
    get_button_color('none', 'enemy_color', 'blue', 'red')

    # ...

# Initialize Global Values
def init_redis_values():

    # Set to Default Value when Server Starts
    redis_client.set('goal_color', 0) # 0: Orange, 1: Yellow
    redis_client.set('enemy_color', 0) # 0: Blue, 1: Red
    redis_client.set('all_mode', 0) # 0: False, 1: True
    redis_client.set('current_names', '') # Empty on Start
    redis_client.set('name_button_colors', '{}') # Empty on Start

    # Reset each blimp's data in Redis to null (empty)
    for name in blimp_names_order:
        redis_client.delete("blimp:" + name)

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
