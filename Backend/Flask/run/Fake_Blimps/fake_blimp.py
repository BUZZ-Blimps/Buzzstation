# Fake Blimp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int64, Float64, Float64MultiArray, Int64MultiArray
import sys
import random
import time

global vision_timeout_test
vision_timeout_test=False

class Blimp(Node):
    def __init__(self, name):

        # Blimp ID
        self.name = name

        # State Machine
        self.state_machine = 0
        self.catches = 0

        # Barometer Reading
        self.barometer_reading = None

        # Vision
        self.vision = True

        # Define Fake Blimp's name
        self.node_name = str(name)

        # Init node
        super().__init__(self.node_name)

        # Allowed Names
        self.catching_blimp_names = ['BurnCream', 'SillyAh', 'Turbo', 'GameChamber', 'FiveGuys', 'SuperBeef']
        self.attack_blimp_names = ['Yoshi', 'Luigi', 'Geoph', 'ThisGuy']

        self.state_dict = ["searching", "approach", "catching", "caught", "goalSearch", "approachGoal", "scoringStart", "shooting", "scored"]

        # To-Do: Test without Identify (Using State Machine from now on; Can still add /identify for basestation lost state)
        # self.pub_identify = self.create_publisher(String, '/identify', 10)

        # Publishers #

        # Heartbeat
        self.pub_heartbeat = self.create_publisher(Bool, self.node_name + '/' + 'heartbeat', 10)

        # State Machine
        if self.name in self.catching_blimp_names:
            self.pub_state_machine = self.create_publisher(Int64MultiArray, self.node_name + '/' + 'state_machine', 10)
        elif self.name in self.attack_blimp_names:
            self.pub_state_machine = self.create_publisher(Bool, self.node_name + '/' + 'state_machine', 10)

        # Miscellaneous
        self.pub_vision = self.create_publisher(Bool, self.node_name + '/' + 'vision', 10)
        self.pub_height = self.create_publisher(Float64, self.node_name + '/' + 'height', 10)
        self.pub_z_velocity = self.create_publisher(Float64, self.node_name + '/' + 'z_velocity', 10)
        self.pub_battery_status = self.create_publisher(Float64MultiArray, self.node_name + '/' + 'battery_status', 10)
        self.pub_log = self.create_publisher(String, self.node_name + '/' + 'log', 10)

        # Subscribers #

        # Catching Blimps
        if self.name in self.catching_blimp_names:
            topic_goal_color = self.node_name + '/' + 'goal_color'
            topic_catching = self.node_name + '/' + 'catching'
            topic_shooting = self.node_name + '/' + 'shooting'
            self.sub_goal_color = self.create_subscription(Bool, topic_goal_color, self.goal_callback, 10)
            self.sub_catching = self.create_subscription(Bool, topic_catching, self.catch_callback, 10)
            self.sub_shooting = self.create_subscription(Bool, topic_shooting, self.shoot_callback, 10)

        # Attacking Blimps
        if self.name in self.attack_blimp_names:
            topic_enemy_color = self.node_name + '/' + 'enemy_color'
            self.sub_enemy_color = self.create_subscription(Bool, topic_enemy_color, self.enemy_callback, 10)

        # Both Catching and Attacking Blimps
        topic_mode = self.node_name + '/' + 'mode'
        topic_vision = self.node_name + '/' + 'vision'
        topic_motor_commands = self.node_name + '/' + 'motor_commands'
        topic_barometer = 'Barometer/reading'
        topic_calibrate_barometer = self.node_name + '/' + 'calibrate_barometer'
        self.sub_mode = self.create_subscription(Bool, topic_mode, self.mode_callback, 10)
        self.sub_vision = self.create_subscription(Bool, topic_vision, self.vision_callback, 10)
        self.sub_motor_commands = self.create_subscription(Float64MultiArray, topic_motor_commands, self.motor_callback, 10)
        self.sub_barometer = self.create_subscription(Float64, topic_barometer, self.barometer_callback, 10)
        self.sub_calibrate_barometer = self.create_subscription(Bool, topic_calibrate_barometer, self.calibrate_barometer_callback, 10)

        # For Testing:
        # Initialize time tracking for vision updates
        self.last_vision_update_time = time.time()
        self.vision_update_interval = 10  # seconds

        # State Machine
        main_loop_period = 0.5  # seconds
        self.main_loop_timer = self.create_timer(main_loop_period, self.main_loop)

        data_timer_period = 1  # seconds
        self.data_loop_timer = self.create_timer(data_timer_period, self.data_loop)

    # Publishers #

    def main_loop(self):

        if self.vision == False:
             # If Vision Code Stops Running, Restart it Automatically Here
             # Put Here
             self.vision = True
             self.publish_vision()

        # For Testing:
        global vision_timeout_test
        if vision_timeout_test == True:
             current_time = time.time()
             # Update vision every 10 seconds
             if current_time - self.last_vision_update_time >= self.vision_update_interval:
                 self.vision = random.choice([True, False])
                 self.publish_vision()
                 self.last_vision_update_time = current_time

        # Publish Heartbeat #
        self.heartbeat = True
        self.publish_heartbeat()

        # Publish State Machine #

        # Catching Blimp
        if self.name in self.catching_blimp_names:
            # self.state_machine = round(random.random()*8)

            if (self.state_machine < 8):
                self.state_machine = self.state_machine + 1
                # Occasionally increment catches when state cycles through
                if self.state_machine == 8 and random.random() > 0.7:
                    self.catches += 1
            else:
                self.state_machine = 0

            print(f"State: {self.state_dict[self.state_machine]}, Catches: {self.catches}")

            self.publish_catching_blimp_state_machine()

        # Attacking Blimp
        elif self.name in self.attack_blimp_names:
            # self.state_machine = random.choice([True, False])
            self.publish_attack_blimp_state_machine()

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = self.heartbeat
        self.pub_heartbeat.publish(msg)

    def publish_catching_blimp_state_machine(self):
        msg = Int64MultiArray()
        msg.data = [self.state_machine, self.catches]
        self.pub_state_machine.publish(msg)

    def publish_attack_blimp_state_machine(self):
        msg = Bool()
        msg.data = self.state_machine
        self.pub_state_machine.publish(msg)

    def publish_vision(self):
        msg = Bool()
        msg.data = self.vision
        self.pub_vision.publish(msg)

    def data_loop(self):
        # Height data
        self.height = 30 * random.random() - 15
        self.z_velocity = 5 * random.random()

        # Battery data - simulate a battery between 7.0-8.0V and 0-100% charge
        battery_voltage = 7.0 + (1.0 * random.random())  # voltage between 7.0-8.0V
        battery_percentage = random.randint(20, 100)  # percentage between 20-100%

        # Create messages
        self.height_msg = Float64()
        self.height_msg.data = self.height

        self.z_velocity_msg = Float64()
        self.z_velocity_msg.data = self.z_velocity

        self.battery_msg = Float64MultiArray()
        self.battery_msg.data = [battery_voltage, float(battery_percentage)]
        print(f"DEBUG: Fake blimp {self.name} publishing battery status: {battery_voltage:.2f}V, {battery_percentage}%")

        # Publish messages
        self.pub_height.publish(self.height_msg)
        self.pub_z_velocity.publish(self.z_velocity_msg)
        self.pub_battery_status.publish(self.battery_msg)
        print(f"DEBUG: Published battery status message for {self.name}")

    # Subscribers #

    def mode_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Mode set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def vision_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Vision set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

        self.vision = msg.data

    def goal_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Goal color set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def enemy_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Enemy color set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def motor_callback(self, msg):
        m = msg.data
        self.get_logger().info('{},{},{},{}'.format(m[0], m[1], m[2], m[3]))

    def catch_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Catch set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def shoot_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Shoot set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

    def barometer_callback(self, msg):
        self.barometer_reading = msg.data

    def calibrate_barometer_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Calibrate set to {}'.format(msg.data)

        # Fake Calibration
        self.height = 0.00
        self.height_msg = Float64()
        self.height_msg.data = self.height
        self.pub_height.publish(self.height_msg)

        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: python3 fake_blimp.py node_name")
        return

    node_name = str(sys.argv[1])

    node = Blimp(node_name)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
