# ========== Publishers ========== #

"""
Description:

ROS 2 Publishers for Basestation to send data to Blimps.

To-Do:

- Implement and Test State Machine Publisher

"""

from Packages.packages import Bool, Int64, Float64, Float64MultiArray

# Create Publishers #

def create_publishers(blimp):

    # Catching Blimp
    if blimp.type is False:

        # Goal Color
        blimp.pub_goal_color = create_pub(blimp, 'goal_color', 'Bool')

        # Catching
        blimp.pub_catching = create_pub(blimp, 'catching', 'Bool')

        # Shooting
        blimp.pub_shooting = create_pub(blimp, 'shooting', 'Bool')

        # State Machine (For changing state of blimp during testing)
        # blimp.pub_state_machine = create_pub(blimp, 'state_machine', 'Int64')
        """
        0: Searching
        1: Approach
        2: Catching
        3: Caught
        4: Goal Search
        5: Approach Goal
        6: Scoring Start
        7: Shooting
        8: Scored
        """

    # Attack Blimp
    else:

        # Enemy Color
        blimp.pub_enemy_color = create_pub(blimp, 'enemy_color', 'Bool')

        # State Machine (For changing state of blimp during testing)
        # blimp.pub_state_machine = create_pub(blimp, 'state_machine', 'Bool')
        """
        False: Searching
        True: Approaching
        """

    # Both Catching and Attack Blimps
    blimp.pub_mode = create_pub(blimp, 'mode', 'Bool')
    blimp.pub_vision = create_pub(blimp, 'vision', 'Bool')
    blimp.pub_motor_commands = create_pub(blimp, 'motor_commands', 'Float64MultiArray')
    blimp.pub_calibrate_barometer = create_pub(blimp, 'calibrate_barometer', 'Bool')

def create_pub(blimp, key, data_type):
    if data_type == 'Bool':
        pub = blimp.basestation_node.create_publisher(Bool, f'{blimp.name}/{key}', blimp.basestation_node.boolean_qos_profile)
    elif data_type == 'Int64':
        pub = blimp.basestation_node.create_publisher(Int64, f'{blimp.name}/{key}', 1)
    elif data_type == 'Float64MultiArray':
        pub = blimp.basestation_node.create_publisher(Float64MultiArray, f'{blimp.name}/{key}', blimp.basestation_node.motor_commands_qos_profile)
    else:
        pass
    return pub

# Destroy Publishers

def destroy_publishers(blimp):

    # Catching Blimp
    if blimp.type is False:

        # Goal Color
        destroy_pub(blimp, 'pub_goal_color')

        # Catching
        destroy_pub(blimp, 'pub_catching')

        # Shooting
        destroy_pub(blimp, 'pub_shooting')

        # State Machine (For changing state of blimp during testing)
        # destroy_pub(blimp, 'pub_state_machine')
        """
        0: Searching
        1: Approach
        2: Catching
        3: Caught
        4: Goal Search
        5: Approach Goal
        6: Scoring Start
        7: Shooting
        8: Scored
        """

    # Attack Blimp
    else:

        # Enemy Color
        destroy_pub(blimp, 'pub_enemy_color')

        # State Machine (For changing state of blimp during testing)
        # destroy_pub(blimp, 'pub_state_machine')
        """
        False: Searching
        True: Approaching
        """

    # Both Catching and Attack Blimps
    destroy_pub(blimp, 'pub_mode')
    destroy_pub(blimp, 'pub_vision')
    destroy_pub(blimp, 'pub_motor_commands')
    destroy_pub(blimp, 'pub_calibrate_barometer')

def destroy_pub(blimp, key):
    blimp.basestation_node.destroy_publisher(getattr(blimp, key))

# Generic Publish Function #
def publish_generic(publish_function_name, blimp):
    # Get the function object based on its name
    publish_function = globals().get(publish_function_name, None)
    if publish_function:
        
        # Call the function with the blimp argument
        publish_function(blimp)

# Barometer Publisher #
def publish_barometer(basestation_node):
    # Publish barometer value to the ROS topic
    msg = Float64()
    msg.data = basestation_node.barometer_reading
    basestation_node.pub_barometer.publish(msg)

# Catching Blimp Publishers #

def publish_goal_color(blimp):
    # Publish goal color value to the ROS topic
    msg = Bool()
    msg.data = blimp.goal_color
    blimp.pub_goal_color.publish(msg)

def publish_catching(blimp):
    # Publish catching value to the ROS topic
    msg = Bool()
    msg.data = blimp.catching
    blimp.pub_catching.publish(msg)

def publish_shooting(blimp):
    # Publish shooting value to the ROS topic
    msg = Bool()
    msg.data = blimp.shooting
    blimp.pub_shooting.publish(msg)

# Attack Blimp Publishers #

def publish_enemy_color(blimp):
    # Publish enemy color value to the ROS topic
    msg = Bool()
    msg.data = blimp.enemy_color
    blimp.pub_enemy_color.publish(msg)

# Both Catching and Attack Blimp Publishers #

def publish_mode(blimp):
    # Publish mode value to the ROS topic
    msg = Bool()
    msg.data = blimp.mode
    blimp.pub_mode.publish(msg)

def publish_motor_commands(blimp):
    # Publish motor_commands value to the ROS topic
    # Different than Blimp Class Motor Commands!
    # Used for to Zero Out Motor Commands for Safety
    msg = Float64MultiArray()
    msg.data = blimp.motor_commands
    blimp.pub_motor_commands.publish(msg)

def publish_calibrate_barometer(blimp):
    # Publish calibrate barometer value to the ROS topic (Only true for one message)
    msg = Bool()
    msg.data = blimp.calibrate_barometer
    blimp.pub_calibrate_barometer.publish(msg)

def publish_vision(blimp):
    # Publish vision value to the ROS topic
    msg = Bool()
    msg.data = blimp.vision
    blimp.pub_vision.publish(msg)
