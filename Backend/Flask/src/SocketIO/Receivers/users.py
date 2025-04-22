# ========== Users ========== #

"""
Description:

SocketIO user events are handled here. User events include user 
connection and user inactivity/disconnection.

"""

# Imports
from Packages.packages import *

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# User Connection
@socketio.on('connect')
def handle_connect():
    from SocketIO.Senders.redis import get_redis_values

    # Log User Connection
    logger.info('User Connected')

    # Get Redis Values upon Connection
    get_redis_values()

# Request for battery status
@socketio.on('request_battery_status')
def handle_request_battery():
    from SocketIO.Senders.redis import get_button_values
    
    # Send battery status values to the requesting client
    get_button_values('battery_status')

# User Inactivity/Disconnection
@socketio.on('inactive_user')
def inactive_user(userID):
    from ROS.ros import basestation_node
    from SocketIO.socketio import name_button_colors

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

                # Stop Motor Commands Timer
                if basestation_node.current_blimps[name].motor_commands_timer is not None:
                    basestation_node.current_blimps[name].motor_commands_timer.cancel()
                    basestation_node.current_blimps[name].motor_commands_timer = None
