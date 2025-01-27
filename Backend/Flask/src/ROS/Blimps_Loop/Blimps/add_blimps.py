# ========== Add Blimps ========== #

"""
Description:

Adds a new blimp not in the current blimps list and creates a new blimp object.

"""

# Imports
from Packages.packages import socketio
from .blimp import Blimp
from .blimp_type import is_attack_blimp
from ...Communication.publishers import publish_generic

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Create blimp objects for new blimp names and add to current blimps
def add_new_blimps(basestation_node, new_blimp_names):
    for blimp_name in new_blimp_names:
        if blimp_name not in basestation_node.current_blimps:

            # Create a new blimp and add to current blimps and blimp names
            new_blimp = Blimp(basestation_node, blimp_name)
            basestation_node.current_blimps[blimp_name] = new_blimp
            basestation_node.current_blimp_names.append(blimp_name)
            logger.info(str('Identified Blimp: ' + new_blimp.name))

            # Send Time since Last Heartbeat to Frontend (0.0 seconds)
            socketio.emit('name_time_since_last_heartbeat',  { 'name': new_blimp.name, 'time_since_last_heartbeat': round(0.0, 2) })

            # Publish Global Values to Initialize (Enemy or Goal Color)
            if is_attack_blimp(blimp_name):
                publish_generic('publish_enemy_color', new_blimp)
            else:
                publish_generic('publish_goal_color', new_blimp)
            
            # Publish Individual Values

            # Mode
            publish_generic('publish_mode', new_blimp)

            # Vision
            publish_generic('publish_vision', new_blimp)
            # To-Do: Startup Blimp Vision Code
            # Put Here

            # Set Calibrate to False
            socketio.emit('update_button_color', {'name': new_blimp.name, 'key': 'calibrate_barometer', 'color': 'red'})

            # Set Catch and Shoot Icons to False
            socketio.emit('toggle_catch_icon',  { 'name': new_blimp.name, 'val': False })
            socketio.emit('toggle_shoot_icon',  { 'name': new_blimp.name, 'val': False })
            