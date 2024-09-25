# ========== Update Blimp Lists ========== #

"""
Description:

- Update the blimp list by checking for alive blimps.
- Removes timeout blimps
- Adds new blimps

"""

# Imports
from Packages.packages import *
from .blimp_names import blimp_names_order

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Main Functions #

# Check for alive blimps with heartbeat data available
def alive_blimps(basestation_node):
    # Get all heartbeat topics
    heartbeat_topics = [topic_name for topic_name, _ in basestation_node.get_topic_names_and_types() if topic_name.endswith('/heartbeat')]

    # Extract namespaces from topic names
    namespaces = {topic_name.split('/')[1] for topic_name in heartbeat_topics}

    alive_blimps = []

    for namespace in namespaces:
        if namespace not in blimp_names_order:
            continue  # Skip if not in blimp order list

        blimp = basestation_node.current_blimps.get(namespace)
        if blimp is None:
            # If first time this blimp is seen, initialize it
            alive_blimps.append(namespace)
            continue

        # Check if heartbeat data is available and within timeout
        current_time = time()
        time_since_last_heartbeat = current_time - blimp.last_heartbeat_time

        # Testing
        #logger.info(blimp.name)
        #logger.info(str(round(time_since_last_heartbeat, 2)))

        # Send Time since Last Heartbeat to Frontend (Usually less than a second if alive)
        socketio.emit('name_time_since_last_heartbeat',  { 'name': blimp.name, 'time_since_last_heartbeat': round(time_since_last_heartbeat, 2) })

        if time_since_last_heartbeat <= basestation_node.heartbeat_timeout:
            blimp.heartbeat_data = None  # Reset heartbeat data
            alive_blimps.append(namespace)
            blimp.last_online = basestation_node.get_clock().now()  # Update last online time

    return alive_blimps

# Make new blimps name list
def new_blimps(alive_blimps, current_blimps):
    old_and_new = find_different_values(alive_blimps, current_blimps)
    new = find_common_values(alive_blimps, old_and_new)
    return new

# Make timeout blimps name list
def timeout_blimps(new_blimps, current_blimps):
    old_and_new = find_different_values(new_blimps, current_blimps)
    old = find_common_values(current_blimps, old_and_new)
    return old

# Reorder Current Blimp Names
def reorder_blimp_names(current_blimp_names):
    # Create a mapping of the base order to use for sorting
    order_map = {value: index for index, value in enumerate(blimp_names_order)}
    
    # Sort the list based on the index from the base_order
    # Defaulting to a large number if the item is not in base_order
    reordered_list = sorted(
        current_blimp_names, 
        key=lambda item: order_map.get(item, float('inf'))
    )
    
    return reordered_list

# Helper Functions #

# Helper function to find common values in two lists
# Returns a list of common values
def find_common_values(list1, list2):
    # Convert lists to sets
    set1 = set(list1)
    set2 = set(list2)
    
    # Find the intersection (common values) between the sets
    common_values = set1 & set2
    
    return list(common_values)

# Helper function to find different values in two lists
# Returns a list of different values
def find_different_values(list1, list2):
    # Convert lists to sets
    set1 = set(list1)
    set2 = set(list2)
    
    # Find the symmetric difference (different values) between the sets
    different_values = set1 ^ set2
    
    return list(different_values)
