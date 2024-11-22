# ========== Subscribers ========== #

"""
Description:

ROS 2 Subscribers for Basestation to recieve data from Blimps.

"""

from Packages.packages import Bool, Int64MultiArray, Float64, String

# Create Subscribers #

def create_subscribers(blimp):
    # Catching Blimp
    if blimp.type is False:

        # State Machine
        blimp.sub_state = create_sub(blimp, 'state', 'Int64MultiArray')

    # Attack Blimp
    else:
        # State Machine
        blimp.sub_state = create_sub(blimp, 'state', 'Float64')

    # Height
    blimp.sub_height = create_sub(blimp, 'height', 'Float64')

    # Z-Velocity
    blimp.sub_z_velocity = create_sub(blimp, 'z_velocity', 'Float64')

    # Vision
    blimp.sub_vision = create_sub(blimp, 'vision', 'Bool')

    # Log
    blimp.sub_log = create_sub(blimp, 'log', 'String')

def create_sub(blimp, key, data_type):
    if key == 'state':
        sub = blimp.basestation_node.create_subscription(Int64MultiArray, f'{blimp.name}/{key}', blimp.state_callback, 10)
    elif key == 'height':
        if data_type == 'Float64':
            sub = blimp.basestation_node.create_subscription(Float64, f'{blimp.name}/{key}', blimp.height_callback, 10)
    elif key == 'z_velocity':
        if data_type == 'Float64':
            sub = blimp.basestation_node.create_subscription(Float64, f'{blimp.name}/{key}', blimp.z_velocity_callback, 10)
    elif key == 'vision':
        if data_type == 'Bool':
            sub = blimp.basestation_node.create_subscription(Bool, f'{blimp.name}/{key}', blimp.vision_callback, 10)
    elif key == 'log':
        if data_type == 'String':
            sub = blimp.basestation_node.create_subscription(String, f'{blimp.name}/{key}', blimp.log_callback, 10)
    return sub

def create_heartbeat_sub(blimp):
    # Subscribe to heartbeat if not already subscribed
    blimp.basestation_node.heartbeat_subs[blimp.name] = blimp.basestation_node.create_subscription(
        Bool, # Data Type
        f"/{blimp.name}/heartbeat", # Topic Name
        blimp.heartbeat_callback, # Callback
        1 # QoS Profile
    )

# Destroy Subscribers #
def destroy_subscribers(blimp):
    # Catching Blimp
    if blimp.type is False:
        # State Machine
        destroy_sub(blimp, 'sub_state')

    # Attack Blimp
    else:
        # State Machine
        destroy_sub(blimp, 'sub_state')

    # Height
    destroy_sub(blimp, 'sub_height')

    # Z-Velocity
    destroy_sub(blimp, 'sub_z_velocity')

    # Vision
    destroy_sub(blimp, 'sub_vision')

    # Log
    destroy_sub(blimp, 'sub_log')

    # Destroy Heartbeat Subscriber and Remove from Heartbeat Subscriber Dictionary
    blimp.basestation_node.destroy_subscription(blimp.basestation_node.heartbeat_subs[blimp.name])
    del blimp.basestation_node.heartbeat_subs[blimp.name]

def destroy_sub(blimp, key):
    blimp.basestation_node.destroy_subscription(getattr(blimp, key))
