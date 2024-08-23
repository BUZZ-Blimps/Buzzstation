# ========== Remove Blimps ========== #

"""
Description:

Removes a blimp that times out from the current blimps list and deles the blimp object.

"""

from Packages.packages import socketio, redis_client, json
from SocketIO.socketio import name_button_colors
from ...Communication.publishers import destroy_publishers
from ...Communication.subscribers import destroy_subscribers

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Remove blimp objects for blimps not in the new blimp names list
def remove_timeout_blimps(basestation_node, timeout_blimp_names):
    for blimp_name in timeout_blimp_names:
        blimp = basestation_node.current_blimps[blimp_name]
        if getElapsedTime(basestation_node, blimp.last_online) > basestation_node.blimp_timeout:
            remove_blimp(basestation_node, blimp)

def remove_blimp(basestation_node, blimp):
    logger.info(str('Detected timeout of Blimp: ' + blimp.name))

    if blimp.name in name_button_colors:
        # Remove Blimp Name and UserID from Dictionary
        del name_button_colors[blimp.name]

        # Store Blimp Button Colors to Redis
        redis_client.set('name_button_colors', json.dumps(name_button_colors))

        # Make Blimp Name Button Green for all users
        socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': blimp.name})
    
    # Destroy Blimp Publishers and Subscribers
    destroy_publishers(blimp)
    destroy_subscribers(blimp)
    
    # Remove from current blimps and blimp names
    del basestation_node.current_blimps[blimp.name]
    basestation_node.current_blimp_names.remove(blimp.name)

    # Destroy Blimp Object
    del blimp

def getElapsedTime(basestation_node, prevTime):
    elapsedTime = basestation_node.get_clock().now() - prevTime
    elapsedTimeSec = elapsedTime.nanoseconds / 10**9
    return elapsedTimeSec
    