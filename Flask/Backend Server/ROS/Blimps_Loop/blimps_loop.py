# ========== Blimps Loop ========== #

"""
Description:

Sends and recieves blimp data over ROS.

"""

# Imports
from ..ros import basestation_node, redis_client
from .Blimps.update_blimp_lists import alive_blimps, new_blimps, timeout_blimps
from .Blimps.update_blimp_data import update_component_for_all_blimps, update_blimp_component_color
from .Blimps.add_blimps import add_new_blimps
from .Blimps.remove_blimps import remove_timeout_blimps

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Blimps Timer Loop
def blimps_loop():

    # Increment Blimps Loop Count
    basestation_node.blimps_loop_count = basestation_node.blimps_loop_count + 1

    # Update Global Values: Goal Color, Enemy Color, All Auto State
    update_global_values()

    # Update Blimps
    update_blimps()

def update_global_values():
    # Goal Color (Default: Orange, Nondefault: Yellow)
    update_component_for_all_blimps(basestation_node, 'goal_color', 'orange', 'yellow') 

    # Enemy Color (Default: Blue, Nondefault: Red)
    update_component_for_all_blimps(basestation_node, 'enemy_color', 'blue', 'red')

    # All Autonomous State (Sends all current blimps into autonomous mode)
    # To-Do: Need to make function in update_blimps_data

def update_blimp_values(current_blimp_names):
    # Loop?
    for blimp_name in current_blimp_names:
        update_blimp_component_color()

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

    # To-Do: Reorder names here
    # reorder_blimp_names(basestation_node, new_blimp_names)

    # Save Current Blimp Names to Redis
    current_blimp_names = ','.join(alive_blimp_names)
    redis_client.set('current_blimp_names', current_blimp_names)

    # Update Blimp Values on Frontend and over ROS
    update_blimp_values(current_blimp_names)
