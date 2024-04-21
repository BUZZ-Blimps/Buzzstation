# ========== Update Blimp Lists ========== #

"""
Description:

"""

# Imports
from Packages.packages import *
from time import time

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Initialize a timestamp to track the last heartbeat message received time
global last_heartbeat_time
last_heartbeat_time = time()

# Heartbeat Data
global heartbeat_data
heartbeat_data = None

# Check for alive blimps with heartbeat data available
def alive_blimps(basestation_node):
    heartbeat_topics = [topic_name for topic_name, _ in basestation_node.get_topic_names_and_types()
                        if topic_name.endswith('/heartbeat')]

    namespaces = []
    for topic_name in heartbeat_topics:
        # Extract namespace from the topic name
        namespace = topic_name.split('/')[1]

        if namespace not in basestation_node.heartbeat_subs:
            heartbeat_sub = basestation_node.create_subscription(Bool, topic_name, check_heartbeat, 10)
            basestation_node.heartbeat_subs[namespace] = heartbeat_sub

        # Check if heartbeat is recent
        if heartbeat_data is not None and time() - last_heartbeat_time <= basestation_node.heartbeat_timeout:
            # Alive Blimp
            namespaces.append(namespace)
            # Blimp is online
            if namespace in basestation_node.current_blimps:
                basestation_node.current_blimps[namespace].last_online = basestation_node.get_clock().now()

    return namespaces

# Heartbeat Callback for checking if a Blimp is still alive
def check_heartbeat(msg):
    # Update last message received time
    global last_heartbeat_time, heartbeat_data
    last_heartbeat_time = time()
    heartbeat_data = msg.data

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