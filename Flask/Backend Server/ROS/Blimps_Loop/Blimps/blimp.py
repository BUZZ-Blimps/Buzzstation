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
            1: Approach
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

        # Autonomous Mode (False: Manual, True: Autonomous)
        self.auto = False

        # Motor Commands
        self.motor_commands = [0.0, -0.0, 0.0, -0.0]

        # Controlled status
        self.controlled = False  # Initially not controlled

        # Barometer Value
        self.barometer = 99668.2 # Competition Default Value

        # Calibrate Barometer
        self.calibrate_barometer = False

        # Height
        self.height = None

        # Z Velocity
        self.z_velocity = None

        # Last Log Message
        self.log = None

        # Last Time State Machine Publishes over ROS
        self.last_online = 0

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
        self.pub_auto = self.create_pub('auto', 'Bool')
        self.pub_motor_commands = self.create_pub('motor_commands', 'Float64MultiArray')
        self.pub_barometer = self.create_pub('barometer', 'Bool')
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
        self.destroy_pub('pub_auto')
        self.destroy_pub('pub_motor_commands')
        self.destroy_pub('pub_barometer')
        self.destroy_pub('pub_calibrate_barometer')

    def destroy_pub(self, key):
        self.basestation_node.destroy_publisher(getattr(self, key))

    # Create and Destroy Subscribers #
    # Note: Maybe Move to subscribers.py and refactor (Test first using rti)
    
    def create_subscribers(self):
        pass

    def create_sub(self, key, data_type):
        pass

    def destroy_subscribers(self):
        # Destroy Heartbeat Subscriber and Remove from Heartbeat Subscriber Dictionary
        self.basestation_node.destroy_subscription(self.basestation_node.heartbeat_subs[self.name])
        del self.basestation_node.heartbeat_subs[self.name]
    
    def destroy_sub(self, key):
        pass
