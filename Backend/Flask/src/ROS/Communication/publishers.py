# ========== Publishers ========== #

"""
Description:

ROS 2 Publishers for Basestation to send data to Blimps.

To-Do:

- Implement and Test State Machine Publisher

"""

from Packages.packages import Bool, Int64, Float64, Float64MultiArray

# Generic Publish Function
def publish_generic(publish_function_name, blimp):
    # Get the function object based on its name
    publish_function = globals().get(publish_function_name, None)
    if publish_function:
        # Call the function with the blimp argument
        publish_function(blimp)

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
    msg = Float64MultiArray()
    msg.data = blimp.motor_commands
    blimp.pub_motor_commands.publish(msg)

def publish_barometer(blimp):
    # Publish barometer value to the ROS topic
    msg = Float64()
    msg.data = float(blimp.barometer)
    blimp.pub_barometer.publish(msg)

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
