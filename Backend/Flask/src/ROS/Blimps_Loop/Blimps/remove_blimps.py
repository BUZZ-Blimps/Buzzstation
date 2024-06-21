# ========== Remove Blimps ========== #

"""
Description:

"""

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
    
    # To-Do: Destroy Blimp Subscribers and Publishers
    blimp.destroy_subscribers()
    blimp.destroy_publishers()
    
    # Remove from current blimps and blimp names
    del basestation_node.current_blimps[blimp.name]
    basestation_node.current_blimp_names.remove(blimp.name)

    # Destroy Blimp Object
    del blimp

def getElapsedTime(basestation_node, prevTime):
    elapsedTime = basestation_node.get_clock().now() - prevTime
    elapsedTimeSec = elapsedTime.nanoseconds / 10**9
    return elapsedTimeSec