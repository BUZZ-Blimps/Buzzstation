# ========== Buttons ========== #

"""
Description:

SocketIO button events are handled here.

"""

# Imports
from Packages.packages import *

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Toggle Name Button with Three Colors
@socketio.on('toggle_name_button')
def toggle_name_button(val):
    from SocketIO.socketio import name_button_colors

    # Retrieve Blimp Name and UserID
    name = val['name']
    userID = val['userID']

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

# Toggle Blimp Buttons with Two Colors
@socketio.on('toggle_blimp_button_color')
def toggle_blimp_button_color(val):
    from ROS.ros import basestation_node

    # Retrieve Blimp Name and Key
    name = val['name']
    key = val['key']

    if hasattr(basestation_node.current_blimps[name], key):

        # Change the Value of the Key for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], key, not getattr(basestation_node.current_blimps[name], key))
        
# Set Blimp Button to a Specific Value
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

# Set Calibrate Barometer for Blimp to True
@socketio.on('toggle_blimp_calibrate_button_color')
def toggle_blimp_calibrate_button_color(name):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    if basestation_node.barometer_serial != None or basestation_node.fake_barometer == True:

        if hasattr(basestation_node.current_blimps[name], 'calibrate_barometer'):

            # Set Calibrate Barometer to True
            setattr(basestation_node.current_blimps[name], 'calibrate_barometer', True)

            # Set Calibrated to True
            setattr(basestation_node.current_blimps[name], 'calibrated', True)
            
            # Update Frontend
            socketio.emit('update_button_color', {'name': name, 'key': 'calibrate_barometer', 'color': 'green'})

            if basestation_node.current_blimps[name].calibrate_barometer:
                
                # Publish over ROS
                publish_generic('publish_calibrate_barometer', basestation_node.current_blimps[name])

                # Set Calibrate Barometer to False
                setattr(basestation_node.current_blimps[name], 'calibrate_barometer', False)

                # Publish over ROS
                publish_generic('publish_calibrate_barometer', basestation_node.current_blimps[name])

# Toggle All Blimps Buttons with Two Colors (i.e. Goal, Enemy, All Auto, All Manual)
@socketio.on('toggle_all_blimps_button_color')
def toggle_all_blimps_button_color(key):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Change and Set Redis Value
    button_color = not button_color
    redis_client.set(key, int(button_color))
