# ========== Main ROS Node ========== #

"""
Description:

Main ROS file where data is sent and recieved by the Backend Server
over ROS 2 using the Rclpy Python package. This file includes two
loops in the main ROS Node called "Basestation". The loops are
the Blimps Loop and Barometer Loop.

Blimps Loop: Sends and recieves blimp data over ROS.

Barometer Loop: Connects to the Barometer via a Serial Port and
send Barometer data over ROS.

To-Do:

- Maybe Add Identify for Blimp Lost State
- Always Publish to Attack Blimps (They can't latch currently due to middleman server issue)

"""

# Imports
from Packages.packages import *

global basestation_node

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Read the YAML file
with open('../src/Config/default_values.yaml', 'r') as file:
    blimp_data = yaml.safe_load(file)

# Basestation Node
class Basestation(Node):
    def __init__(self):
        super().__init__('Basestation')

        # ROS Joy (Controller)
        if blimp_data.get('ros_joy') is True:
            from .Joy.ros_joy import init_ros_joy
            logger.info("ROS JOY Enabled")
            init_ros_joy(self)

        # Initialize attributes for Basestation Node
        self.init_attributes()

        # Create Loop for Blimp ROS Data
        self.create_blimps_loop()

    def init_attributes(self):
        
        # Global Basestation Node Class (Used in other files)
        global basestation_node
        basestation_node = self

        # Current Blimps Objects
        self.current_blimps = {}

        # Current Blimp Names
        self.current_blimp_names = []

        # QoS Profile for Boolean Latches
        self.boolean_qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # QoS Profile for Motor Commands
        self.motor_commands_qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Redis Client
        self.redis_client = redis_client

        # Global Values
        self.goal_color = bool(int(self.redis_client.get('goal_color').decode('utf-8')))
        self.enemy_color = bool(int(self.redis_client.get('enemy_color').decode('utf-8')))

    def create_blimps_loop(self):

        # Heartbeat Subscriptions
        self.heartbeat_subs = {}

        # Heartbeat Timeout
        self.heartbeat_timeout = 4.0

        # Blimps Loop Speed
        self.blimps_loop_speed = 5 # (Hz) Depends on CPU Speed
        
        # Blimps Loop Period
        blimps_loop_period = 1.0/self.blimps_loop_speed
        
        # Blimps Loop Count
        self.blimps_loop_count = 0

        # Blimps Loop Timeout (Total Timeout = Heartbeat Timeout + Blimp Timeout)
        self.blimp_timeout = 1.0
        
        # Create Blimps Loop Timer
        from .Blimps_Loop.blimps_loop import blimps_loop
        self.blimps_timer = self.create_timer(blimps_loop_period, blimps_loop)

        # Motor Commands Publisher Frequency
        self.motor_commands_frequency = 100 # (Hz) for each blimp being controlled

        # Faking Barometer Serial Port Connection
        self.fake_barometer = blimp_data.get('fake_barometer')

        # Barometer Publisher
        self.sub_barometer = self.create_subscription(Float64, 'Barometer/reading', self.barometer_callback, 10)

        # # Barometer Reading
        self.barometer_reading = 0
        self.barometer_online = False
        self.barometer_time = time()

        # Barometer Reading
        self.barometer_reading = blimp_data.get('barometer')

        # Barometer Online
        self.barometer_online = True

        # Barometer Time
        self.barometer_time = time()

        # Barometer Loop Speed
        self.barometer_loop_speed = 5 # (Hz) Depends on CPU Speed

        # # Barometer Loop Period
        barometer_timer_period = 1.0/self.barometer_loop_speed

        from .Barometer_Loop.barometer_loop import barometer_loop
        self.barometer_timer = self.create_timer(barometer_timer_period, barometer_loop)

    def barometer_callback(self, msg):
        self.barometer_time = time()
        self.barometer_reading = msg.data
        self.barometer_online = True

    # Barometer Callback
    def barometer_callback(self, msg):
        self.barometer_reading = msg.data
        self.barometer_time = time()
        self.barometer_online = True

# ROS 2 Thread
def ros_node():

    rclpy.init()

    global node

    node = Basestation()
    
    # To-Do: Maybe don't need (Throwing error when used)
    #traceback_logger = rclpy.logging.get_logger('basestation_traceback_logger')

    try:
        rclpy.spin(node)
    except Exception as error:
        pass
        # To-Do: Maybe don't need (Throwing error when used)
        #traceback_logger.error(traceback.format_exc())

    try:
        node.destroy_node()
    except rclpy.handle.InvalidHandle:
        pass

    try:
        rclpy.shutdown()
    except Exception as e:
        pass

# Start ROS 2
def start_ros():

    # Create and Start ROS 2 Thread
    ros_thread = threading.Thread(target=ros_node)
    ros_thread.start()
