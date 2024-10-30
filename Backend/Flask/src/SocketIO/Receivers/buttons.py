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
    from ROS.ros import basestation_node

    # Retrieve Blimp Name and UserID
    name = val['name']
    userID = val['userID']

    # Turn on Piloting for Specified User (Connect to Blimp)
    if name not in name_button_colors:
        
        # Only allow each user to pilot one blimp
        if userID not in name_button_colors.values():

            # Add Blimp Name and UserID to Dictionary
            name_button_colors[name] = userID

            # Make Blimp Button Blue for UserID, Red for everyone else
            socketio.emit('toggle_name_button_color', { 'userID': userID, 'name': name})

            # Start Motor Commands Timer
            basestation_node.current_blimps[name].motor_commands_timer = basestation_node.create_timer(float(1.0 / 100), basestation_node.current_blimps[name].publish_motor_commands)

    # Turn off Piloting (Disconnect from blimp)
    elif name_button_colors[name] == userID:

        # Remove Blimp Name and UserID from Dictionary
        del name_button_colors[name]

        # Make Blimp Name Button Green for all users
        socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

        # Stop Motor Commands Timer
        if basestation_node.current_blimps[name].motor_commands_timer is not None:
            basestation_node.current_blimps[name].motor_commands_timer.cancel()
            basestation_node.current_blimps[name].motor_commands_timer = None

    # Store Blimp Button Colors to Redis
    redis_client.set('name_button_colors', json.dumps(name_button_colors))

# Toggle Blimp Buttons with Two Colors
@socketio.on('toggle_blimp_button_color')
def toggle_blimp_button_color(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    # Retrieve Blimp Name and Key
    name = val['name']
    key = val['key']

    if hasattr(basestation_node.current_blimps[name], key):

        # Change the Value of the Key for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], key, not getattr(basestation_node.current_blimps[name], key))

        if key == 'mode':

            # Set Catching to False if currently True
            if hasattr(basestation_node.current_blimps[name], 'catching'):

                if basestation_node.current_blimps[name].mode is True and basestation_node.current_blimps[name].catching is True:

                    # Change the Value of the Catching for the Specific Blimp Name
                    basestation_node.current_blimps[name].catching = not basestation_node.current_blimps[name].catching
                    publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])

                    # Toggle Catch Icon
                    socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })

            # Set Shooting to False if currently True
            if hasattr(basestation_node.current_blimps[name], 'shooting'):

                if basestation_node.current_blimps[name].mode is True and basestation_node.current_blimps[name].shooting is True:

                    # Change the Value of the Shooting for the Specific Blimp Name
                    basestation_node.current_blimps[name].shooting = not basestation_node.current_blimps[name].shooting
                    publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                    # Toggle Shoot Icon
                    socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })
        
# Set Blimp Button to a Specific Value (i.e. All Auto, All Manual)
@socketio.on('set_blimp_button_value')
def set_blimp_button_value(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    # Retrieve Blimp Name, Key, and Value
    name = val['name']
    key = val['key']
    value = bool(int(val['value']))

    if hasattr(basestation_node.current_blimps[name], key):

        # Change the Value of the Key for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], key, value)

        if key == 'mode':

            # Set Catching to False if currently True
            if hasattr(basestation_node.current_blimps[name], 'catching'):

                if basestation_node.current_blimps[name].mode is True and basestation_node.current_blimps[name].catching is True:

                    # Change the Value of the Catching for the Specific Blimp Name
                    basestation_node.current_blimps[name].catching = not basestation_node.current_blimps[name].catching
                    publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])

                    # Toggle Catch Icon
                    socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })

            # Set Shooting to False if currently True
            if hasattr(basestation_node.current_blimps[name], 'shooting'):

                if basestation_node.current_blimps[name].mode is True and basestation_node.current_blimps[name].shooting is True:

                    # Change the Value of the Shooting for the Specific Blimp Name
                    basestation_node.current_blimps[name].shooting = not basestation_node.current_blimps[name].shooting
                    publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                    # Toggle Shoot Icon
                    socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })
        

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

# Toggle All Blimps Buttons with Two Colors (i.e. Goal, Enemy)
@socketio.on('toggle_all_blimps_button_color')
def toggle_all_blimps_button_color(key):

    # Get Redis Value
    button_color = bool(int(redis_client.get(key).decode('utf-8')))

    # Change and Set Redis Value
    button_color = not button_color
    redis_client.set(key, int(button_color))

    # Update Yaml Value
    update_yaml_value(key, int(button_color))

# Update Yaml Value for the Default Values (Goal and Enemy Colors)
def update_yaml_value(key, new_value):
    # Step 1: Read the entire file content
    with open('../src/Config/default_values.yaml', 'r') as file:
        lines = file.readlines()

    # Step 2: Find the line with the key and replace the value
    with open('../src/Config/default_values.yaml', 'w') as file:
        for line in lines:
            # Check if the line starts with the key (ignoring spaces) and contains ':'
            if line.strip().startswith(f"{key}:"):
                # Replace the old value with the new value while keeping the key and colon
                line = f"{key}: {new_value}\n"
            file.write(line)
