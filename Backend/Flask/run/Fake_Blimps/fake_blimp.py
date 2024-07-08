import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int64, Float64, Float64MultiArray
import sys
import random

class Blimp(Node):
    def __init__(self, name):

        # Blimp ID
        self.name = name

        # State Machine
        self.state_machine = 0

        # Define Fake Blimp's name
        self.node_name = str(name)

        # Init node
        super().__init__(self.node_name, namespace=self.node_name)

        # Allowed Names
        self.catching_blimp_names = ['BurnCream', 'SillyAh', 'Turbo', 'GameChamber', 'FiveGuys', 'SuperBeef']
        self.attack_blimp_names = ['Yoshi', 'Luigi', 'Geoph', 'ThisGuy']

        # To-Do: Test without Identify (Using State Machine from now on; Can still add /identify for basestation lost state)
        # self.pub_identify = self.create_publisher(String, '/identify', 10)

        # Publishers #

        # Heartbeat
        self.pub_heartbeat = self.create_publisher(Bool, 'heartbeat', 10)

        # State Machine
        if self.name in self.catching_blimp_names:
            self.pub_state_machine = self.create_publisher(Int64, 'state_machine', 10)
        elif self.name in self.attack_blimp_names:
            self.pub_state_machine = self.create_publisher(Bool, 'state_machine', 10)

        # Miscellaneous
        self.pub_height = self.create_publisher(Float64, 'height', 10)
        self.pub_z_velocity = self.create_publisher(Float64, 'z_velocity', 10)
        self.pub_log = self.create_publisher(String, 'log', 10)

        # Subscribers #

        # Catching Blimps
        if self.name in self.catching_blimp_names:
            topic_goal_color = "goal_color"
            topic_catching = "catching"
            topic_shooting = "shooting"
            self.sub_goal_color = self.create_subscription(Bool, topic_goal_color, self.goal_callback, 10)
            self.sub_catching = self.create_subscription(Bool, topic_catching, self.catch_callback, 10)
            self.sub_shooting = self.create_subscription(Bool, topic_shooting, self.shoot_callback, 10)

        # Attacking Blimps
        if self.name in self.attack_blimp_names:
            topic_enemy_color = "enemy_color"
            self.sub_enemy_color = self.create_subscription(Bool, topic_enemy_color, self.enemy_callback, 10)

        # Both Catching and Attacking Blimps
        topic_mode = "mode"
        topic_motor_commands = "motor_commands"
        topic_barometer = "barometer"
        topic_calibrate_barometer = "calibrate_barometer"
        self.sub_mode = self.create_subscription(Bool, topic_mode, self.mode_callback, 10)
        self.sub_motor_commands = self.create_subscription(Float64MultiArray, topic_motor_commands, self.motor_callback, 10)
        self.sub_barometer = self.create_subscription(Float64, topic_barometer, self.barometer_callback, 10)
        self.sub_calibrate_barometer = self.create_subscription(Bool, topic_calibrate_barometer, self.calibrate_barometer_callback, 10)

        # State Machine
        main_loop_period = 1  # seconds
        self.main_loop_timer = self.create_timer(main_loop_period, self.main_loop)

        data_timer_period = 1/20.0  # seconds
        self.data_loop_timer = self.create_timer(data_timer_period, self.data_loop)

    # Publishers #

    def main_loop(self):
        # Publish Heartbeat #
        self.heartbeat = True
        self.publish_heartbeat()

        # Publish State Machine #

        # Catching Blimp
        if self.name in self.catching_blimp_names:
            self.state_machine = round(random.random()*8)
            self.publish_catching_blimp_state_machine()

        # Attacking Blimp
        elif self.name in self.attack_blimp_names:
            self.state_machine = random.choice([True, False])
            self.publish_attack_blimp_state_machine()

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = self.heartbeat
        self.pub_heartbeat.publish(msg)

    def publish_catching_blimp_state_machine(self):
        msg = Int64()
        msg.data = self.state_machine
        self.pub_state_machine.publish(msg)

    def publish_attack_blimp_state_machine(self):
        msg = Bool()
        msg.data = self.state_machine
        self.pub_state_machine.publish(msg)

    def data_loop(self):
        height = 10*random.random()
        z_velocity = 5*random.random()

        height_msg = Float64()
        height_msg.data = height

        z_velocity_msg = Float64()
        z_velocity_msg.data = z_velocity

        self.pub_height.publish(height_msg)
        self.pub_z_velocity.publish(z_velocity_msg)

    # Subscribers #

    def mode_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Mode set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

        self.get_logger().info(log_msg.data)

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

    def shoot_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Shoot set to {}'.format(msg.data)
        self.pub_log.publish(log_msg)

    def barometer_callback(self, msg):
        pass

    def calibrate_barometer_callback(self, msg):
        log_msg = String()
        log_msg.data = 'Calibrate set to {}'.format(msg.data)

        self.pub_log.publish(log_msg)

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
