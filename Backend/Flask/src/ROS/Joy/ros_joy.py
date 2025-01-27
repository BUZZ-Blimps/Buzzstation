# ROS Joy #

"""
Description:

ROS Joy polls for controller input for the device running this backend program.

To-Do:

- Latch and Save Motor Command Data to Redis

"""

from Packages.packages import Joy, socketio

def init_ros_joy(basestation_node):
    #Subscribe to joystick topic
    basestation_node.joy_sub = basestation_node.create_subscription(Joy, "/Basestation/joy", joy_callback, 1)

    basestation_node.joy_state = Joy()
    basestation_node.joy_state.axes = [0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0]
    basestation_node.joy_state.buttons = [0,0,0,0,0,0,0,0,0,0,0,0]

    basestation_node.axis_labels = [
        'Left Joy R/L',
        'Left Joy U/D',
        'L trigger',
        'Rt Joy R/L',
        'Rt Joy U/D',
        'R trigger',
        'Dpad R/L',
        'Dpad U/D'
    ]

    basestation_node.button_labels = [
        "A",
        "B",
        "X",
        "Y",
        "L bumper",
        "R bumper",
        "Windows",
        "Three Bars",
        "Unused",
        "Left Joy Press",
        "Right Joy Press",
        "Arrow out of box"
    ]

def joy_callback(msg):
    from ..ros import basestation_node
    # Axes:
    #Joysticks
    for i in [0,1,3,4]:
        if msg.axes[i] != basestation_node.joy_state.axes[i]:
            basestation_node.joy_state.axes[i] = msg.axes[i]
            # self.get_logger().info('{} moved'.format(self.axis_labels[i]))

    # 1: Left Joy U/D
    if msg.axes[1] != basestation_node.joy_state.axes[1]:
        basestation_node.joy_state.axes[1] = msg.axes[1]
        # self.get_logger().info('Left joystick U/D')

    # 2: Left Trigger
    if msg.axes[2] != basestation_node.joy_state.axes[2]:
        if basestation_node.joy_state.axes[2] > 0.5 and msg.axes[2] < -0.5:
            #Pressed down
            basestation_node.joy_state.axes[2] = msg.axes[2]
            # self.get_logger().info('Left trigger pressed')
        elif basestation_node.joy_state.axes[2] < -0.5 and msg.axes[2] > 0.5:
            #Released
            # self.get_logger().info('Left trigger released')
            #basestation_node.update_auto_panic()
            basestation_node.joy_state.axes[2] = msg.axes[2]

    # 5: Right Trigger
    if msg.axes[5] != basestation_node.joy_state.axes[5]:
        if basestation_node.joy_state.axes[5] > 0.5 and msg.axes[5] < -0.5:
            #Left trigger pressed down
            basestation_node.joy_state.axes[5] = msg.axes[5]
        elif basestation_node.joy_state.axes[5] < -0.5 and msg.axes[5] > 0.5:
            #Released
            # self.get_logger().info('Right trigger released')
            #Toggle autonomous
            #basestation_node.update_auto()
            basestation_node.joy_state.axes[5] = msg.axes[5]

    if msg.axes[6] != basestation_node.joy_state.axes[6]:
        if msg.axes[6] == 0:
            if basestation_node.joy_state.axes[6] == 1:
                # self.get_logger().info('Left DPad released')
                #basestation_node.disconnect_controller()
                pass
            else:
                # self.get_logger().info('Right DPad released')
                #basestation_node.control_first_blimp()
                pass
        basestation_node.joy_state.axes[6] = msg.axes[6]

    # 7: Dpad U/D
    if msg.axes[7] != basestation_node.joy_state.axes[7]:
        if msg.axes[7] == 0:
            #Update with the negative of what was sent bc of top-down list
            #basestation_node.update_selected_blimp(-self.joy_state.axes[7])
            pass

        basestation_node.joy_state.axes[7] = msg.axes[7]

    for i in range(12):
        if msg.buttons[i] != basestation_node.joy_state.buttons[i]:
            #Button released
            if msg.buttons[i] == 0:
                basestation_node.get_logger().info('{} button released'.format(basestation_node.button_labels[i]))
                # self.button_labels = [
                #     "A",
                #     "B",
                #     "X",
                #     "Y",
                #     "L bumper",
                #     "R bumper",
                #     "Windows",
                #     "Three Bars",
                #     "Unused",
                #     "Left Joy Press",
                #     "Right Joy Press",
                #     "Arrow out of box"
                # ]
                if i == 0:
                    # A button - reload page via emit
                    #socketio.emit('reload')
                    pass
                elif i == 1:
                    # B button
                    pass
                elif i == 2:
                    # X button - Enemy Color Toggle
                    #self.update_enemy_color()
                    pass
                elif i == 3:
                    # Y button - Goal Color Toggle
                    #self.update_goal_color()
                    pass
                elif i == 4:
                    # L bumper - Shoot
                    #self.update_shoot()
                    pass
                elif i == 5:
                    # R bumper - Catch
                    #self.update_grab()
                    pass
                elif i == 11:
                    # Kill Backend
                    basestation_node.get_logger().info('Killing the base station')

            basestation_node.joy_state.buttons[i] = msg.buttons[i]

    #Update the motor commands every time a joy update is received
    update_motor_commands(basestation_node)

def update_motor_commands(basestation_node):
    # To-Do: Latch this and use Redis, then publish over ROS if a blimp is selected (To make more efficent)
    left_stick_x = -basestation_node.joy_state.axes[0]
    left_stick_y = basestation_node.joy_state.axes[1]
    right_stick_x = basestation_node.joy_state.axes[3]
    right_stick_y = basestation_node.joy_state.axes[4]
    controller_cmd = [left_stick_x, left_stick_y, right_stick_x, right_stick_y]
    # Emit Motor Commands to Frontend
    socketio.emit('motor_commands', controller_cmd)

    # for blimp_node_handler in basestation_node.blimp_node_handlers:
    #     if blimp_node_handler.blimp.id == basestation_node.selected_blimp_id:
    #         left_stick_x = -basestation_node.joy_state.axes[0]
    #         left_stick_y = basestation_node.joy_state.axes[1]
    #         right_stick_x = basestation_node.joy_state.axes[3]
    #         right_stick_y = basestation_node.joy_state.axes[4]
    #         controller_cmd = [left_stick_x, left_stick_y, right_stick_x, right_stick_y]
    #         # Emit Motor Commands to Frontend
    #         socketio.emit('motor_commands', controller_cmd)
    #         blimp_node_handler.blimp.motor_commands = controller_cmd
    #         blimp_node_handler.publish_motor_commands()
    #     else:
    #         blimp_node_handler.blimp.motor_commands = [0.0, 0.0, 0.0, 0.0]
