# ========== Subscribers ========== #

"""
Description:

ROS 2 Subscribers for Basestation to recieve data from Blimps.

"""

from Packages.packages import Bool, Int64, Float64, String

# Create Subscribers #

def create_subscribers(blimp):
    # Catching Blimp
    if blimp.type is False:

        # State Machine
        blimp.sub_state_machine = create_sub(blimp, 'state_machine', 'Int64')

    # Attack Blimp
    else:

        # State Machine
        blimp.sub_state_machine = create_sub(blimp, 'state_machine', 'Bool')

    # Height
    blimp.sub_height = create_sub(blimp, 'height', 'Float64')

    # Z-Velocity
    blimp.sub_z_velocity = create_sub(blimp, 'z_velocity', 'Float64')

    # Vision
    blimp.sub_vision = create_sub(blimp, 'vision', 'Bool')

    # Log
    blimp.sub_log = create_sub(blimp, 'log', 'String')

def create_sub(blimp, key, data_type):
    if key == 'state_machine':
        if data_type == 'Bool':
            sub = blimp.basestation_node.create_subscription(Bool, f'{blimp.name}/{key}', blimp.state_machine_callback, 10)
        elif data_type == 'Int64':
            sub = blimp.basestation_node.create_subscription(Int64, f'{blimp.name}/{key}', blimp.state_machine_callback, 10)
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
        destroy_sub(blimp, 'sub_state_machine')

    # Attack Blimp
    else:

        # State Machine
        destroy_sub(blimp, 'sub_state_machine')

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