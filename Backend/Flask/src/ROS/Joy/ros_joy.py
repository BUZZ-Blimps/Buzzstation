# ROS Joy #

"""
Description:

ROS Joy receives controller input from the joy_node running on the device
running this backend program (see run/basestation.launch.py). Joystick
values are written to the motor commands of the blimp piloted by the
controller, and button releases are handled in joy_buttons.py.

The controller pilots as a single user (JOY_USER_ID). A blimp is piloted
by the controller when its name is selected in the UI or with the D-pad.

Joy Mapping (Xbox Controller, joy_node on Linux):

Axes: 0 Left Joy R/L, 1 Left Joy U/D, 2 L Trigger, 3 Rt Joy R/L,
      4 Rt Joy U/D, 5 R Trigger, 6 Dpad R/L, 7 Dpad U/D
      (Left and Up are positive, Triggers rest at 1.0 and are -1.0 when pressed)

Buttons: 0 A, 1 B, 2 X, 3 Y, 4 L Bumper, 5 R Bumper, 6 View,
         7 Menu, 8 Xbox, 9 Left Joy Press, 10 Right Joy Press, 11 Share

"""

from Packages.packages import Joy, socketio, time

# User ID that the controller pilots blimps as
JOY_USER_ID = 'joy'

# Joy Button Index -> Button Name used by joy_buttons.py
JOY_BUTTONS = {
    0: 'button0',   # A
    1: 'button1',   # B
    2: 'button2',   # X
    3: 'button3',   # Y
    4: 'button4',   # Left Bumper (LB)
    5: 'button5',   # Right Bumper (RB)
    6: 'button8',   # View
    7: 'button9',   # Menu
    8: 'button16',  # Xbox
    9: 'button10',  # Left Stick (LS)
    10: 'button11', # Right Stick (RS)
}

# Trigger Axis Index -> Button Name
JOY_TRIGGERS = {
    2: 'button6',   # Left Trigger (LT)
    5: 'button7',   # Right Trigger (RT)
}

# D-pad Axis Index -> (Button Name when Positive, Button Name when Negative)
JOY_DPAD = {
    6: ('button14', 'button15'),  # Left, Right
    7: ('button12', 'button13'),  # Up, Down
}

# Zero Motor Commands if no Joy Message is received for this long (joy_node autorepeats at 20 Hz)
JOY_TIMEOUT = 0.5 # (s)

def init_ros_joy(basestation_node):
    #Subscribe to joystick topic
    basestation_node.joy_sub = basestation_node.create_subscription(Joy, "/Basestation/joy", joy_callback, 1)

    # Last Joy Message
    basestation_node.joy_state = None

    # Last Joy Message Time
    basestation_node.joy_time = None

    # Controller Timeout Check (Zero Motor Commands if Controller Disconnects)
    basestation_node.joy_timeout_timer = basestation_node.create_timer(0.1, joy_timeout_check)

def joy_callback(msg):
    from ..ros import basestation_node

    prev = basestation_node.joy_state
    basestation_node.joy_state = msg
    basestation_node.joy_time = time()

    if prev is not None:
        for button in get_released_buttons(prev, msg):
            handle_button(button)

    update_motor_commands(basestation_node)

def get_released_buttons(prev, msg):
    released = []

    # Buttons
    for i, button in JOY_BUTTONS.items():
        if i < len(msg.buttons) and i < len(prev.buttons):
            if prev.buttons[i] == 1 and msg.buttons[i] == 0:
                released.append(button)

    # Triggers (Released when going from pressed down to let go)
    for i, button in JOY_TRIGGERS.items():
        if i < len(msg.axes) and i < len(prev.axes):
            if prev.axes[i] < -0.5 and msg.axes[i] > 0.5:
                released.append(button)

    # D-pad
    for i, (positive_button, negative_button) in JOY_DPAD.items():
        if i < len(msg.axes) and i < len(prev.axes):
            if prev.axes[i] != 0 and msg.axes[i] == 0:
                released.append(positive_button if prev.axes[i] > 0 else negative_button)

    return released

def handle_button(button):
    from ..ros import basestation_node
    from SocketIO.socketio import name_button_colors
    from .joy_buttons import handle_blimp_button, handle_nonblimp_button

    # Frontend Only Buttons
    if button == 'button8':
        # Toggle Controller Mapping Overlay Image
        socketio.emit('toggle_overlay_image')
    elif button == 'button9':
        # Toggle Sidebar Menu
        socketio.emit('toggle_sidebar_menu')

    joy_blimp_names = get_joy_blimp_names(basestation_node, name_button_colors)

    if joy_blimp_names:
        for name in joy_blimp_names:
            handle_blimp_button(name, button, JOY_USER_ID)
    else:
        handle_nonblimp_button(button, JOY_USER_ID)

def get_joy_blimp_names(basestation_node, name_button_colors):
    return [name for name in basestation_node.current_blimp_names if name_button_colors.get(name) == JOY_USER_ID]

def get_controller_cmd(msg):
    # Right and Up are positive
    left_stick_x = -msg.axes[0]
    left_stick_y = msg.axes[1]
    right_stick_x = -msg.axes[3]
    right_stick_y = msg.axes[4]
    return [float(left_stick_x), float(left_stick_y), float(right_stick_x), float(right_stick_y)]

def update_motor_commands(basestation_node, controller_cmd=None):
    from SocketIO.socketio import name_button_colors

    if controller_cmd is None:
        controller_cmd = get_controller_cmd(basestation_node.joy_state)

    # Emit Motor Commands to Frontend (Controller Input Display)
    socketio.emit('motor_commands', controller_cmd)

    # Motor Commands are published by each piloted blimp's motor commands timer
    for name in get_joy_blimp_names(basestation_node, name_button_colors):
        blimp = basestation_node.current_blimps[name]
        if blimp.mode is False:
            blimp.motor_commands = list(controller_cmd)

def joy_timeout_check():
    from ..ros import basestation_node

    if basestation_node.joy_time is not None and time() - basestation_node.joy_time > JOY_TIMEOUT:
        basestation_node.get_logger().warn('Controller timed out, zeroing motor commands')
        basestation_node.joy_state = None
        basestation_node.joy_time = None
        update_motor_commands(basestation_node, [0.0, 0.0, 0.0, 0.0])
