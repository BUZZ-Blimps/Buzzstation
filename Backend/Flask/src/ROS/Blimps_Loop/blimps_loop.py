# ========== Blimps Loop ========== #

"""
Description:

Sends and recieves blimp data over ROS.

"""

# Imports
from Packages.packages import socketio
from ..ros import basestation_node, redis_client
from .Blimps.update_blimp_names import alive_blimps, new_blimps, timeout_blimps, reorder_blimp_names
from .Blimps.update_blimp_data import update_component_for_all_blimps, update_blimp_component_color, update_blimp_component_value
from .Blimps.add_blimps import add_new_blimps
from .Blimps.remove_blimps import remove_timeout_blimps

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Blimps Timer Loop
def blimps_loop():

    # Increment Blimps Loop Count
    basestation_node.blimps_loop_count = basestation_node.blimps_loop_count + 1

    # Update Global Values: Goal Color, Enemy Color
    update_global_values()

    # Update Blimps
    update_blimps()

def update_global_values():
    # Goal Color (Default: Orange, Nondefault: Yellow)
    update_component_for_all_blimps(basestation_node, 'goal_color', 'orange', 'yellow') 

    # Enemy Color (Default: Blue, Nondefault: Red)
    update_component_for_all_blimps(basestation_node, 'enemy_color', 'blue', 'red')

# To-Do: Finish this function
def update_blimp_values(current_blimps):
    # Update Individual Blimp Values
    for name in current_blimps:
        
        # State
        update_blimp_component_value(current_blimps[name], 'state_machine')

        # Mode
        update_blimp_component_color(current_blimps[name], 'mode', 'red', 'green')

        # Height
        update_blimp_component_value(current_blimps[name], 'height')

        # Vision
        update_blimp_component_color(current_blimps[name], 'vision', 'green', 'red')

def update_blimps():

    # Alive Blimp Names
    alive_blimp_names = alive_blimps(basestation_node)

    # New Blimp Names
    new_blimp_names = new_blimps(alive_blimp_names, basestation_node.current_blimp_names)

    # Timeout Blimp Names
    timeout_blimp_names = timeout_blimps(alive_blimp_names, basestation_node.current_blimp_names)

    # Remove Timeout Blimps
    remove_timeout_blimps(basestation_node, timeout_blimp_names)

    # Handle New Blimps
    add_new_blimps(basestation_node, new_blimp_names)

    # Reorder Blimp Names
    alive_blimp_names = reorder_blimp_names(alive_blimp_names)
    basestation_node.current_blimp_names = reorder_blimp_names(basestation_node.current_blimp_names)

    # Get Current Blimp Names from Redis
    current_blimp_names = redis_client.get('current_names').decode("utf-8")

    # If Blimp Names Changed, Update Frontend
    if ','.join(basestation_node.current_blimp_names) != current_blimp_names:

        # Save Current Blimp Names to Redis
        current_blimp_names = ','.join(basestation_node.current_blimp_names)
        redis_client.set('current_names', current_blimp_names)

        # Update Blimp Names on Frontend
        socketio.emit('update_names', current_blimp_names.split(','))

        logger.info('Blimp Names Changed')

    # Update Blimp Values on Frontend and over ROS
    update_blimp_values(basestation_node.current_blimps)

    # To-Do: Make this latched by comparing to most recent data if changed #
    # Blimps Dictionary
    blimps_dict = {blimp.name: blimp.to_dict() for blimp in basestation_node.current_blimps.values()}
    
    # Store each blimp's data in Redis using HMSET
    for blimp_name, blimp_data in blimps_dict.items():
        redis_client.hmset(str("blimp:" + blimp_name), blimp_data)
    # End of To-Do #
