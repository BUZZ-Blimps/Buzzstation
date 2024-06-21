# ========== Add Blimps ========== #

"""
Description:

"""

# Imports
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
            new_blimp = Blimp(basestation_node, blimp_name)
            basestation_node.current_blimps[blimp_name] = new_blimp
            basestation_node.current_blimp_names.append(blimp_name)
            logger.info(str('Identified Blimp: ' + new_blimp.name))

            # Publish Global Values to Initialize (Enemy or Goal Color)
            if is_attack_blimp(blimp_name):
                publish_generic('publish_enemy_color', new_blimp)
            else:
                publish_generic('publish_goal_color', new_blimp)
