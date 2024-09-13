# ========== Redis ========== #

"""
Description:

Redis value initialization and retrieval functions. After retrieval,
the values are sent to the user's frontend.

To-Do:

- For the function init_redis_values, use a yaml file to read 
initial goal_color, enemy_color, and barometer values 
(Last Instance from previous program run)

"""

# Imports
from Packages.packages import *

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Initialize Redis Values
def init_redis_values():
    from ROS.Blimps_Loop.Blimps.blimp_names import blimp_names_order

    # Set to Default Value when Server Starts

    # Read the YAML file
    with open('../src/Config/default_values.yaml', 'r') as file:
        blimp_data = yaml.safe_load(file)

    redis_client.set('goal_color', blimp_data.get('goal_color'))
    redis_client.set('enemy_color', blimp_data.get('enemy_color'))
    redis_client.set('barometer', blimp_data.get('barometer'))
    
    redis_client.set('current_names', '') # Empty on Start
    redis_client.set('name_button_colors', '{}') # Empty on Start

    # Reset each blimp's data in Redis to null (empty)
    for name in blimp_names_order:
        redis_client.delete("blimp:" + name)

# Get Redis Values and Sends to Frontend UI
def get_redis_values():

    # List of all Redis Values #

    # Names
    get_names()

    # Name Button Colors
    get_name_button_colors()

    # State Machine Values
    get_button_values('state_machine')

    # Mode Button Colors
    get_mode_button_colors()

    # Calibrate Button Colors
    get_calibrate_button_colors()

    # Height Values
    get_button_values('height')

    # Vision Button Colors
    get_vision_button_colors()

    # Goal Color Button (Default: Orange, Nondefault: Yellow)
    get_button_color('none', 'goal_color', 'orange', 'yellow')
    
    # Enemy Color Button (Default: Blue, Nondefault: Red)
    get_button_color('none', 'enemy_color', 'blue', 'red')

    # ...

# Get a Color for any Button
def get_button_color(name, key, default_color, nondefault_color):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Send Update to Frontend
    if (button_color == False):
        socketio.emit('update_button_color', {'name': name, 'key': key, 'color': default_color})
    else:
        socketio.emit('update_button_color', {'name': name, 'key': key, 'color': nondefault_color})

# Get Value for Each Blimp Button (i.e. State Machine, Height, Z-Velocity, Log, etc.)
def get_button_values(component):

    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if redis_client.hget(str('blimp:' + name), component) is not None:
            current_component_value = redis_client.hget(str('blimp:' + name), component).decode('utf-8')
            # Send Update to Frontend
            socketio.emit('update_button_value', {'name': name, 'key': component, 'value': current_component_value})

# Get Current Blimp Names
def get_names():
    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    # Update Blimp Names on Frontend
    socketio.emit('update_names', current_names.split(','))

# Get Blimp Button Colors for Each Blimp
def get_name_button_colors():
    from SocketIO.socketio import name_button_colors

    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if name in name_button_colors:
            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_name_button_color', { 'userID': name_button_colors[name], 'name': name})
        else:
             # Make Blimp Button Green for all users
            socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

# Get Mode Button Color for Each Blimp
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

# Get Calibrate Button Color for Each Blimp
def get_calibrate_button_colors():

    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if redis_client.hget(str('blimp:' + name), 'calibrated') is not None:
            current_component_color = redis_client.hget(str('blimp:' + name), 'calibrated').decode('utf-8')
            # Send Update to Frontend
            if str(current_component_color) == 'True':
                socketio.emit('update_button_color', {'name': name, 'key': 'calibrate_barometer', 'color': 'green'})
            else:
                socketio.emit('update_button_color', {'name': name, 'key': 'calibrate_barometer', 'color': 'red'})

# Get Vision Button Color for Each Blimp
def get_vision_button_colors():

    # Get Current Blimp Names from Redis
    current_names = redis_client.get('current_names').decode("utf-8")

    for name in current_names.split(','):
        if redis_client.hget(str('blimp:' + name), 'vision') is not None:
            current_component_color = redis_client.hget(str('blimp:' + name), 'vision').decode('utf-8')
            # Send Update to Frontend
            if str(current_component_color) == 'True':
                socketio.emit('update_button_color', {'name': name, 'key': 'vision', 'color': 'green'})
            else:
                socketio.emit('update_button_color', {'name': name, 'key': 'vision', 'color': 'red'})
