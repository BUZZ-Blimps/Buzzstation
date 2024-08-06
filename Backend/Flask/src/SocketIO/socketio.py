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

# User Connection to Backend
@socketio.on('inactive_user')
def inactive_user(userID):
    from ROS.ros import basestation_node

    # Testing
    #logger.info('User Inactive')

    # Disconnect from any selected blimps for User ID
    for name in basestation_node.current_blimp_names:
        if name in name_button_colors:
            if name_button_colors[name] == userID:

                # Remove Blimp Name and UserID from Dictionary
                del name_button_colors[name]

                # Make Blimp Name Button Green for all users
                socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

                # Store Blimp Button Colors to Redis
                redis_client.set('name_button_colors', json.dumps(name_button_colors))

# Get Blimp Motor Command
@socketio.on('motor_command')
def get_blimp_motor_command(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    # Retrieve Blimp Name and Key
    name = val['name']
    axes = val['axes']

    axes[0] = float(axes[0])
    axes[1] = float(axes[1] * -1) # Invert
    axes[2] = float(axes[2])
    axes[3] = float(axes[3] * -1) # Invert

    if hasattr(basestation_node.current_blimps[name], 'motor_commands'):

        # Change the Value of the Motor Commands for the Specific Blimp Name
        setattr(basestation_node.current_blimps[name], 'motor_commands', axes)
        publish_generic('publish_' + 'motor_commands', basestation_node.current_blimps[name])

# Get Blimp Button Release
@socketio.on('blimp_button')
def get_blimp_button_release(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    # Retrieve Blimp Name, Key, and User ID
    name = val['name']
    button = val['button']
    userID = val['userID']

    # A
    if button == 'button0':
        # Reload Page / Toggle or Activate the currently selected button on the screen
        socketio.emit('reload_page', userID)
    
    # B
    elif button == 'button1':
        if hasattr(basestation_node.current_blimps[name], 'vision'):

            # Toggle the Value of the Vision to Manual for the Specific Blimp Name
            basestation_node.current_blimps[name].vision = not basestation_node.current_blimps[name].vision
    
    # X
    elif button == 'button2':
        if hasattr(basestation_node.current_blimps[name], 'mode'):

            # Toggle the Value of the Mode to Auto for the Specific Blimp Name
            basestation_node.current_blimps[name].mode = not basestation_node.current_blimps[name].mode
    
    # Y (To-Do: Individual Calibrate)
    elif button == 'button3':
        # Calibrate Height of Specific Blimp Name
        pass
        # if hasattr(basestation_node.current_blimps[name], 'calibrate_barometer'):

        #     # Set the Value of the Calibrate Barometer to True
        #     basestation_node.current_blimps[name].calibrate_barometer = not basestation_node.current_blimps[name].calibrate_barometer
    
    # Left Bumper (LB)
    elif button == 'button4':
        if hasattr(basestation_node.current_blimps[name], 'catching'):

            # Change the Value of the Catching for the Specific Blimp Name
            basestation_node.current_blimps[name].catching = not basestation_node.current_blimps[name].catching
            publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])
    
    # Right Bumper (RB)
    elif button == 'button5':
        if hasattr(basestation_node.current_blimps[name], 'shooting'):

            # Change the Value of the Shooting for the Specific Blimp Name
            basestation_node.current_blimps[name].shooting = not basestation_node.current_blimps[name].shooting
            publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])
    
    # Left Trigger (LT)
    elif button == 'button6':
        # All Manual
        [setattr(basestation_node.current_blimps[name], 'mode', False) for name in basestation_node.current_blimps]
    
    # Right Trigger (RT)
    elif button == 'button7':
        # All Auto
        [setattr(basestation_node.current_blimps[name], 'mode', True) for name in basestation_node.current_blimps]
    
    # View
    elif button == 'button8':
        # Turn on All Vision
        [setattr(basestation_node.current_blimps[name], 'vision', True) for name in basestation_node.current_blimps]

    # Menu (To-Do: Open Sidebar Menu)
    elif button == 'button9':
        # Open Sidebar Menu (Changes D-Pad, A Button, and B Button Functionality)
        pass
    
    # Left Stick (LS) (Unused)
    elif button == 'button10':
        pass
    
    # Right Stick (RS) (Unused)
    elif button == 'button11':
        pass
    
    # Up (D-pad)
    elif button == 'button12':
        # Switch to Blimp Above or Disconnect

        # Start at the Current Name
        try:
            start_index = basestation_node.current_blimp_names.index(name)
        except:
            start_index = len(basestation_node.current_blimp_names)-1  # Default to end of the list if not found

        # Iterate through the list in reverse, starting at the start index and looping around
        blimp_names = basestation_node.current_blimp_names
        for i in range(len(blimp_names)):
            blimp_name = blimp_names[(start_index - i) % len(blimp_names)]

            # Turn on Piloting for Specified User
            if blimp_name not in name_button_colors:
                # Remove the old blimp name if exists
                if name in name_button_colors:
                    del name_button_colors[name]

                # Store Blimp Button Colors to Redis
                redis_client.set('name_button_colors', json.dumps(name_button_colors))

                # Make Blimp Name Button Green for all users
                socketio.emit('toggle_name_button_color', {'userID': 'none', 'name': name})

                # Only allow each user to pilot one blimp
                if userID not in name_button_colors.values():
                    # Add Blimp Name and UserID to Dictionary
                    name_button_colors[blimp_name] = userID

                    # Store Blimp Button Colors to Redis
                    redis_client.set('name_button_colors', json.dumps(name_button_colors))

                    # Make Blimp Button Blue for UserID, Red for everyone else
                    socketio.emit('toggle_name_button_color', {'userID': userID, 'name': blimp_name})

                    # Break out of the Loop
                    break
    
    # Down (D-pad)
    elif button == 'button13':
        # Switch to Blimp Below or Disconnect

        # Start at the Current Name
        try:
            start_index = basestation_node.current_blimp_names.index(name)
        except:
            start_index = 0

        # Iterate through the list, starting at the start index and looping around
        blimp_names = basestation_node.current_blimp_names
        for i in range(len(blimp_names)):
            blimp_name = blimp_names[(start_index + i) % len(blimp_names)]

            # Turn on Piloting for Specified User
            if blimp_name not in name_button_colors:
                # Remove the old blimp name if exists
                if name in name_button_colors:
                    del name_button_colors[name]

                # Store Blimp Button Colors to Redis
                redis_client.set('name_button_colors', json.dumps(name_button_colors))

                # Make Blimp Name Button Green for all users
                socketio.emit('toggle_name_button_color', {'userID': 'none', 'name': name})

                # Only allow each user to pilot one blimp
                if userID not in name_button_colors.values():
                    # Add Blimp Name and UserID to Dictionary
                    name_button_colors[blimp_name] = userID

                    # Store Blimp Button Colors to Redis
                    redis_client.set('name_button_colors', json.dumps(name_button_colors))

                    # Make Blimp Button Blue for UserID, Red for everyone else
                    socketio.emit('toggle_name_button_color', {'userID': userID, 'name': blimp_name})

                    # Break out of the Loop
                    break
    
    # Left (D-pad)
    elif button == 'button14':
        # Disconnect from any currently selected Blimp Name

        if name_button_colors[name] == userID:

            # Remove Blimp Name and UserID from Dictionary
            del name_button_colors[name]

            # Store Blimp Button Colors to Redis
            redis_client.set('name_button_colors', json.dumps(name_button_colors))

            # Make Blimp Name Button Green for all users
            socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

    # Right (D-pad)
    elif button == 'button15':
        pass
    
    # Xbox
    elif button == 'button16':
        # Kill Basestation (Backend)
        logger.info('Killing Backend')

        # Simulate sending the SIGINT signal to the current process
        os.kill(os.getpid(), signal.SIGINT)

# Get Nonblimp Button Release
@socketio.on('nonblimp_button')
def get_nonblimp_button_release(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic

    # Retrieve Key and User ID
    button = val['button']
    userID = val['userID']

    # A
    if button == 'button0':
        # Reload Page / Toggle or Activate the currently selected button on the screen
        socketio.emit('reload_page', userID)
    
    # B (Unused)
    elif button == 'button1':
        pass
    
    # X (Unused)
    elif button == 'button2':
        pass
    
    # Y (Unused)
    elif button == 'button3':
        pass
    
    # Left Bumper (LB) (Unused)
    elif button == 'button4':
        pass

    # Right Bumper (RB) (Unused)
    elif button == 'button5':
        pass

    # Left Trigger (LT)
    elif button == 'button6':
        # All Manual
        [setattr(basestation_node.current_blimps[name], 'mode', False) for name in basestation_node.current_blimps]
    
    # Right Trigger (RT)
    elif button == 'button7':
        # All Auto
        [setattr(basestation_node.current_blimps[name], 'mode', True) for name in basestation_node.current_blimps]
    
    # View
    elif button == 'button8':
        # Turn on All Vision
        [setattr(basestation_node.current_blimps[name], 'vision', True) for name in basestation_node.current_blimps]

    # Menu (To-Do: Open Sidebar Menu)
    elif button == 'button9':
        # Open Sidebar Menu
        pass
    
    # Left Stick (LS) (Unused)
    elif button == 'button10':
        pass
    
    # Right Stick (RS) (Unused)
    elif button == 'button11':
        pass
    
    # Up (D-pad)
    elif button == 'button12':
        # Bottom Blimp
        
        # Iterate backwards through the list, starting at the last index
        for blimp_name in reversed(basestation_node.current_blimp_names):

            # Turn on Piloting for Specified User
            if blimp_name not in name_button_colors:
                
                # Only allow each user to pilot one blimp
                if userID not in name_button_colors.values():

                    # Add Blimp Name and UserID to Dictionary
                    name_button_colors[blimp_name] = userID

                    # Store Blimp Button Colors to Redis
                    redis_client.set('name_button_colors', json.dumps(name_button_colors))

                    # Make Blimp Button Blue for UserID, Red for everyone else
                    socketio.emit('toggle_name_button_color', { 'userID': userID, 'name': blimp_name})
    
                    # Break out of the Loop
                    break

    # Down (D-pad)
    elif button == 'button13':
        # Top Blimp
        
        # Iterate through the list, starting at the first index
        for blimp_name in basestation_node.current_blimp_names:

            # Turn on Piloting for Specified User
            if blimp_name not in name_button_colors:
                
                # Only allow each user to pilot one blimp
                if userID not in name_button_colors.values():

                    # Add Blimp Name and UserID to Dictionary
                    name_button_colors[blimp_name] = userID

                    # Store Blimp Button Colors to Redis
                    redis_client.set('name_button_colors', json.dumps(name_button_colors))

                    # Make Blimp Button Blue for UserID, Red for everyone else
                    socketio.emit('toggle_name_button_color', { 'userID': userID, 'name': blimp_name})
    
                    # Break out of the Loop
                    break

    # Left (D-pad)
    elif button == 'button14':
        pass

    # Right (D-pad)
    elif button == 'button15':
        # Top Blimp
        
        # Iterate through the list, starting at the first index
        for blimp_name in basestation_node.current_blimp_names:

            # Turn on Piloting for Specified User
            if blimp_name not in name_button_colors:
                
                # Only allow each user to pilot one blimp
                if userID not in name_button_colors.values():

                    # Add Blimp Name and UserID to Dictionary
                    name_button_colors[blimp_name] = userID

                    # Store Blimp Button Colors to Redis
                    redis_client.set('name_button_colors', json.dumps(name_button_colors))

                    # Make Blimp Button Blue for UserID, Red for everyone else
                    socketio.emit('toggle_name_button_color', { 'userID': userID, 'name': blimp_name})
    
                    # Break out of the Loop
                    break

    # Xbox
    elif button == 'button16':
        # Kill Basestation (Backend)
        logger.info('Killing Backend')

        # Simulate sending the SIGINT signal to the current process
        os.kill(os.getpid(), signal.SIGINT)

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

# Get Vision for Each Blimp
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

# Get Redis Values and Sends to Frontend UI
def get_redis_values():

    # List of all Redis Values #

    # Names
    get_names()

    # Name Button Colors
    get_name_button_colors()

    # Mode Button Colors
    get_mode_button_colors()

    # Calibrate Button Colors
    #get_calibrate_button_colors()

    # Vision Button Colors
    get_vision_button_colors()

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
    redis_client.set('current_names', '') # Empty on Start
    redis_client.set('name_button_colors', '{}') # Empty on Start

    # Reset each blimp's data in Redis to null (empty)
    for name in blimp_names_order:
        redis_client.delete("blimp:" + name)

# Start Backend Server
def start_backend_server(host, port):

    # Start App at the specified IP and port
    try:
        socketio.run(app, host=host, port=port, ssl_context='adhoc')
    except:
        try:
            socketio.run(app, allow_unsafe_werkzeug=True, host=host, port=port, ssl_context='adhoc')
        except:
            pass
