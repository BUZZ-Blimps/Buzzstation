# ========== Controller ========== #

"""
Description:

SocketIO controller events are handled here. This include motor commands 
and controller button releases for when the user is connected or not 
connected to a blimp.

"""

# Imports
from Packages.packages import *

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

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

        if basestation_node.current_blimps[name].mode is False:

            # Change the Value of the Motor Commands for the Specific Blimp Name
            setattr(basestation_node.current_blimps[name], 'motor_commands', axes)
            publish_generic('publish_' + 'motor_commands', basestation_node.current_blimps[name])

# Get Blimp Button Release
@socketio.on('blimp_button')
def get_blimp_button_release(val):
    from ROS.ros import basestation_node
    from ROS.Communication.publishers import publish_generic
    from SocketIO.socketio import name_button_colors
    from SocketIO.Receivers.buttons import toggle_blimp_calibrate_button_color

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

            # Zero out motor commands, Turn off shooting and catching if in autonomous mode
            if basestation_node.current_blimps[name].mode is True:

                if hasattr(basestation_node.current_blimps[name], 'motor_commands'):

                    # Change the Value of the Motor Commands for the Specific Blimp Name
                    setattr(basestation_node.current_blimps[name], 'motor_commands', [float(0.0), float(0.0), float(0.0), float(0.0)])
                    publish_generic('publish_' + 'motor_commands', basestation_node.current_blimps[name])
                
                if hasattr(basestation_node.current_blimps[name], 'shooting'):

                    if basestation_node.current_blimps[name].shooting is True:
                        
                        # Set Shooting to False
                        basestation_node.current_blimps[name].shooting = False
                        publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                        # Toggle Shoot Icon
                        socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })
    

                if hasattr(basestation_node.current_blimps[name], 'catching'):

                    if basestation_node.current_blimps[name].catching is True:
                        
                        # Set Catching to False
                        basestation_node.current_blimps[name].catching = False
                        publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])

                        # Toggle Catch Icon
                        socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })

    # Y
    elif button == 'button3':
        # Calibrate Height of Specific Blimp Name
        toggle_blimp_calibrate_button_color(name)
        
    # Left Bumper (LB)
    elif button == 'button4':
        if hasattr(basestation_node.current_blimps[name], 'catching'):

            if basestation_node.current_blimps[name].mode is False:

                # Change the Value of the Catching for the Specific Blimp Name
                basestation_node.current_blimps[name].catching = not basestation_node.current_blimps[name].catching
                publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])

                # Toggle Catch Icon
                socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })
    
    # Right Bumper (RB)
    elif button == 'button5':
        if hasattr(basestation_node.current_blimps[name], 'shooting'):

            if basestation_node.current_blimps[name].mode is False:

                # Change the Value of the Shooting for the Specific Blimp Name
                basestation_node.current_blimps[name].shooting = not basestation_node.current_blimps[name].shooting
                publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                # Toggle Shoot Icon
                socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })
    
    # Left Trigger (LT)
    elif button == 'button6':
        # All Manual
        [setattr(basestation_node.current_blimps[name], 'mode', False) for name in basestation_node.current_blimps]
    
    # Right Trigger (RT)
    elif button == 'button7':
        # All Auto
        [setattr(basestation_node.current_blimps[name], 'mode', True) for name in basestation_node.current_blimps]

        # Zero out motor commands, Turn off shooting and catching
        for name in basestation_node.current_blimps:

            if hasattr(basestation_node.current_blimps[name], 'motor_commands'):

                # Change the Value of the Motor Commands for the Specific Blimp Name
                setattr(basestation_node.current_blimps[name], 'motor_commands', [float(0.0), float(0.0), float(0.0), float(0.0)])
                publish_generic('publish_' + 'motor_commands', basestation_node.current_blimps[name])

            if hasattr(basestation_node.current_blimps[name], 'shooting'):

                if basestation_node.current_blimps[name].shooting is True:
                    
                    # Set Shooting to False
                    basestation_node.current_blimps[name].shooting = False
                    publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                    # Toggle Shoot Icon
                    socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })

            if hasattr(basestation_node.current_blimps[name], 'catching'):

                if basestation_node.current_blimps[name].catching is True:
                    
                    # Set Catching to False
                    basestation_node.current_blimps[name].catching = False
                    publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])
    
                    # Toggle Catch Icon
                    socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })

    # View
    elif button == 'button8':
        # Toggle Controller Mapping Overlay Image (Done on Frontend)
        pass

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
    from SocketIO.socketio import name_button_colors

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

        # Zero out motor commands, Turn off shooting and catching
        for name in basestation_node.current_blimps:

            if hasattr(basestation_node.current_blimps[name], 'motor_commands'):

                # Change the Value of the Motor Commands for the Specific Blimp Name
                setattr(basestation_node.current_blimps[name], 'motor_commands', [float(0.0), float(0.0), float(0.0), float(0.0)])
                publish_generic('publish_' + 'motor_commands', basestation_node.current_blimps[name])

            if hasattr(basestation_node.current_blimps[name], 'shooting'):

                if basestation_node.current_blimps[name].shooting is True:
                    
                    # Set Shooting to False
                    basestation_node.current_blimps[name].shooting = False
                    publish_generic('publish_' + 'shooting', basestation_node.current_blimps[name])

                    # Toggle Shoot Icon
                    socketio.emit('toggle_shoot_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].shooting })

            if hasattr(basestation_node.current_blimps[name], 'catching'):

                if basestation_node.current_blimps[name].catching is True:
                    
                    # Set Catching to False
                    basestation_node.current_blimps[name].catching = False
                    publish_generic('publish_' + 'catching', basestation_node.current_blimps[name])
    
                    # Toggle Catch Icon
                    socketio.emit('toggle_catch_icon',  { 'name': name, 'val': basestation_node.current_blimps[name].catching })

    # View
    elif button == 'button8':
        # Toggle Controller Mapping Overlay Image (Done on Frontend)
        pass

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
