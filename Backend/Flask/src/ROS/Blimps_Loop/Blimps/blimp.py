# ========== Blimp Object Class ========== #

"""
Description:

Blimp Object for Catching and Attacking Blimps. Initializes default values from Redis Database,
and creates and destroys publishers/subscribers for ROS data for that blimp's
namespace.

To-Do: 

- Refactor and Integrate with old BlimpNodeHandler class
- Separate some of this code into different files (file too large right now)
- Publish to State Machine (Would need to change how I am checking for alive blimps potentially; Might have to do anyway because of state machine subscriber)

"""

# Imports
from Packages.packages import *
from time import time
from .blimp_type import is_attack_blimp

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

class Blimp:
    def __init__(self, basestation_node, name):

        # Initialize all Blimp Values
        self.init_values(basestation_node, name)

        # Create Publishers for all Blimp Topics
        self.create_publishers()

        # Create Subscribers for all Blimp Topics
        self.create_subscribers()

    # Initialize Blimp Values
    def init_values(self, basestation_node, name):

        # Basestation Node
        self.basestation_node = basestation_node

        # Blimp Name
        self.name = name

        # Blimp Type (False: Catching, True: Attacking)
        self.type = is_attack_blimp(self.name)

        # Catching Blimp
        if self.type is False:

            # Goal Color
            self.goal_color = self.get_value('goal_color', 'Bool')

            # Catching
            self.catching = False

            # Shooting
            self.shooting = False

            # State Machine
            self.state_machine = 0
            """
            0: Searching
            1: Approaching
            2: Catching
            3: Caught
            4: Goal Search
            5: Approach Goal
            6: Scoring Start
            7: Shooting
            8: Scored
            """

        # Attacking Blimp
        else:

            # Enemy Color
            self.enemy_color = self.get_value('enemy_color', 'Bool')

            # State Machine
            self.state_machine = False
            """
            False: Searching
            True: Approaching
            """

        # Both Catching and Attacking Blimps

        # Mode (False: Manual, True: Autonomous)
        self.mode = False

        # Motor Commands
        self.motor_commands = [float(0.0), float(0.0), float(0.0), float(0.0)]

        # Calibrate Barometer
        self.calibrate_barometer = False

        # Height
        self.height = None

        # Z Velocity
        self.z_velocity = None

        # Vision (False: Off, True: On)
        self.vision = True # Default: True (Initialize Blimp Vision on Startup)

        # Last Log Message
        self.log = None

        # Last Time Heartbeat Publishes over ROS
        self.last_online = 0.0

        # Heartbeat Time last Received
        self.last_heartbeat_time = time()

        # Heartbeat Data
        self.heartbeat_data = None

        self.init_heartbeat_sub()

    def to_dict(self):
        base_dict = {
            'name': str(self.name),
            'type': str(self.type),
            'state_machine': str(self.state_machine),
            'mode': str(self.mode),
            'vision': str(self.vision),
            'motor_commands': str(self.motor_commands),
            'calibrate_barometer': str(self.calibrate_barometer),
            'height': str(self.height),
            'z_velocity': str(self.z_velocity),
            'log': str(self.log),
        }

        # Catching blimp
        if not self.type:
            catching_dict = {
                'catching': str(self.catching),
                'shooting': str(self.shooting),
                'goal_color': str(self.goal_color),
            }
            base_dict.update(catching_dict)
        # Attacking blimp
        else:
            attacking_dict = {
                'enemy_color': str(self.enemy_color),
            }
            base_dict.update(attacking_dict)

        return base_dict

    def init_heartbeat_sub(self):
        # Subscribe to heartbeat if not already subscribed
        self.basestation_node.heartbeat_subs[self.name] = self.basestation_node.create_subscription(
            Bool, # Data Type
            f"/{self.name}/heartbeat", # Topic Name
            self.check_heartbeat, # Callback
            1 # QoS Profile
        )

    # Heartbeat Callback for checking if a Blimp is still alive
    def check_heartbeat(self, msg):
        # Update last message received time
        self.last_heartbeat_time = time()
        self.heartbeat_data = msg.data

    # Get Redis Values
    def get_value(self, key, data_type):
        if data_type == 'Bool':
            value = bool(int(redis_client.get(key).decode('utf-8')))
        elif data_type == 'Int64':
            value = int(redis_client.get(key).decode('utf-8'))
        elif data_type == 'Float64MultiArray':
            # To-Do: Test and Fix this if needed
            value = redis_client.get(key).decode('utf-8')
        else:
            pass
        return value

    # Create and Destroy Publishers #
    # Note: Maybe Move to publishers.py and refactor (Test first using rti and rte)
    
    def create_publishers(self):

        # Catching Blimp
        if self.type is False:

            # Goal Color
            self.pub_goal_color = self.create_pub('goal_color', 'Bool')

            # Catching
            self.pub_catching = self.create_pub('catching', 'Bool')

            # Shooting
            self.pub_shooting = self.create_pub('shooting', 'Bool')

            # State Machine (For changing state of blimp during testing)
            self.pub_state_machine = self.create_pub('state_machine', 'Int64')
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
            self.pub_enemy_color = self.create_pub('enemy_color', 'Bool')

            # State Machine (For changing state of blimp during testing)
            self.pub_state_machine = self.create_pub('state_machine', 'Bool')
            """
            False: Searching
            True: Approaching
            """

        # Both Catching and Attack Blimps
        self.pub_mode = self.create_pub('mode', 'Bool')
        self.pub_vision = self.create_pub('vision', 'Bool')
        self.pub_motor_commands = self.create_pub('motor_commands', 'Float64MultiArray')
        self.pub_calibrate_barometer = self.create_pub('calibrate_barometer', 'Bool')

    def create_pub(self, key, data_type):
        if data_type == 'Bool':
            pub = self.basestation_node.create_publisher(Bool, f'{self.name}/{key}', self.basestation_node.boolean_qos_profile)
        elif data_type == 'Int64':
            pub = self.basestation_node.create_publisher(Int64, f'{self.name}/{key}', 1)
        elif data_type == 'Float64MultiArray':
            pub = self.basestation_node.create_publisher(Float64MultiArray, f'{self.name}/{key}', 1)
        else:
            pass
        return pub
    
    def destroy_publishers(self):

        # Catching Blimp
        if self.type is False:

            # Goal Color
            self.destroy_pub('pub_goal_color')

            # Catching
            self.destroy_pub('pub_catching')

            # Shooting
            self.destroy_pub('pub_shooting')

            # State Machine (For changing state of blimp during testing)
            self.destroy_pub('pub_state_machine')
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
            self.destroy_pub('pub_enemy_color')

            # State Machine (For changing state of blimp during testing)
            self.destroy_pub('pub_state_machine')
            """
            False: Searching
            True: Approaching
            """

        # Both Catching and Attack Blimps
        self.destroy_pub('pub_mode')
        self.destroy_pub('pub_vision')
        self.destroy_pub('pub_motor_commands')
        self.destroy_pub('pub_calibrate_barometer')

    def destroy_pub(self, key):
        self.basestation_node.destroy_publisher(getattr(self, key))

    # Create and Destroy Subscribers #
    # Note: Maybe Move to subscribers.py and refactor (Test first using rti)
    
    def create_subscribers(self):
        # Catching Blimp
        if self.type is False:

            # State Machine
            self.sub_state_machine = self.create_sub('state_machine', 'Int64')

        # Attack Blimp
        else:

            # State Machine
            self.sub_state_machine = self.create_sub('state_machine', 'Bool')

        # Vision
        self.sub_vision = self.create_sub('vision', 'Bool')

    def create_sub(self, key, data_type):
        if key == 'state_machine':
            if data_type == 'Bool':
                sub = self.basestation_node.create_subscription(Bool, f'{self.name}/{key}', self.state_machine_callback, 10)
            elif data_type == 'Int64':
                sub = self.basestation_node.create_subscription(Int64, f'{self.name}/{key}', self.state_machine_callback, 10)
        elif key == 'vision':
            if data_type == 'Bool':
                sub = self.basestation_node.create_subscription(Bool, f'{self.name}/{key}', self.vision_callback, 10)
        return sub

    def destroy_subscribers(self):
        # Catching Blimp
        if self.type is False:

            # State Machine
            self.destroy_sub('sub_state_machine')

        # Attack Blimp
        else:

            # State Machine
            self.destroy_sub('sub_state_machine')

        # Vision
        self.destroy_sub('sub_vision')

        # Destroy Heartbeat Subscriber and Remove from Heartbeat Subscriber Dictionary
        self.basestation_node.destroy_subscription(self.basestation_node.heartbeat_subs[self.name])
        del self.basestation_node.heartbeat_subs[self.name]
    
    def destroy_sub(self, key):
        self.basestation_node.destroy_subscription(getattr(self, key))

    # Blimp Subscriber Callbacks

    def state_machine_callback(self, msg):
        self.state_machine = msg.data

    def vision_callback(self, msg):
        self.vision = msg.data
