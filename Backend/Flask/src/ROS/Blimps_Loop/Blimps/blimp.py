# ========== Blimp Object Class ========== #

"""
Description:

Blimp Object for Catching and Attacking Blimps. Initializes default values from Redis Database,
and creates and destroys publishers/subscribers for ROS data for that blimp's
namespace.

To-Do: 

- Publish to State Machine (Would need to change how I am checking for alive blimps potentially; Might have to do anyway because of state machine subscriber)

"""

# Imports
from Packages.packages import *
from .blimp_type import is_attack_blimp
from ...Communication.publishers import create_publishers
from ...Communication.subscribers import create_subscribers, create_heartbeat_sub

class Blimp:
    def __init__(self, basestation_node, name):

        # Initialize all Blimp Values
        self.init_values(basestation_node, name)

        # Create Publishers for all Blimp Topics
        create_publishers(self)

        # Create Subscribers for all Blimp Topics
        create_subscribers(self)

        self.state_dict = ["searching", "approach", "catching", "caught", "goalSearch", "approachGoal", "scoringStart", "shooting", "scored"]

    # Initialize Blimp Values
    def init_values(self, basestation_node, name):

        # Basestation Node
        self.basestation_node = basestation_node

        # Blimp Name
        self.name = name

        # Blimp Type (False: Catching, True: Attacking)
        self.type = is_attack_blimp(self.name)

        # Catching Blimp #
        
        if self.type is False:

            # Goal Color
            self.goal_color = self.get_redis_value('goal_color', 'Bool')

            # Catching
            self.catching = False

            # Shooting
            self.shooting = False

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

        # Attacking Blimp #
        
        else:

            # Enemy Color
            self.enemy_color = self.get_redis_value('enemy_color', 'Bool')
            """
            False: Searching
            True: Approaching
            """

        # Both Catching and Attacking Blimps #

        # Mode (False: Manual, True: Autonomous)
        self.mode = False

        # State Machine
        self.state_machine = 0

        # Number of catches
        self.catches = 0

        # Motor Commands
        self.motor_commands = [float(0.0), float(0.0), float(0.0), float(0.0)]

        # Motor Commands Timer
        self.motor_commands_timer = None

        # Motor Commands Updated (Not Used Currently)
        self.motor_commands_updated = True

        # Calibrate Barometer
        self.calibrate_barometer = False

        # Calibrated
        self.calibrated = False

        # Height
        self.height = None

        # Z Velocity
        self.z_velocity = None

        # Battery Status
        self.battery_status = None

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

        create_heartbeat_sub(self)

    # Make Blimp Data JSON Readable for Redis
    def to_dict(self):
        base_dict = {
            'name': self.name,
            'type': self.type,
            'state_machine': self.state_machine,
            'catches': self.catches,
            'mode': self.mode,
            'vision': self.vision,
            'motor_commands': self.motor_commands,
            'calibrate_barometer': self.calibrate_barometer,
            'calibrated': self.calibrated,
            'height': self.height,
            'z_velocity': self.z_velocity,
            'battery_status': self.battery_status,
            'log': self.log,
        }

        # Catching blimp
        if not self.type:
            catching_dict = {
                'catching': self.catching,
                'shooting': self.shooting,
                'goal_color': self.goal_color,
            }
            base_dict.update(catching_dict)
        # Attacking blimp
        else:
            attacking_dict = {
                'enemy_color': self.enemy_color,
            }
            base_dict.update(attacking_dict)

        return base_dict

    # Get Redis Value
    def get_redis_value(self, key, data_type):
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

    # Blimp Subscriber Callbacks #

    # Heartbeat Callback 
    # For checking if a Blimp is still alive
    def heartbeat_callback(self, msg):
        # Update last message received time
        self.last_heartbeat_time = time()
        self.heartbeat_data = msg.data

    # State Machine Callback
    def state_callback(self, msg):
        self.state_machine = msg.data[0]
        self.catches = msg.data[1]

    # Height Callback
    def height_callback(self, msg):
        self.height = round(msg.data, 2)
        self.height = str(f"{self.height:.2f}") + 'm'

    # Z-Velocity Callback
    def z_velocity_callback(self, msg):
        self.z_velocity = round(msg.data, 2)
        self.z_velocity = str(f"{self.z_velocity:.2f}") + 'm/s'

    # Battery Status Callback
    def battery_status_callback(self, msg):
        voltage = round(msg.data[0], 2)
        percentage = round(msg.data[1], 2)
        self.battery_status = f"{voltage},{percentage}"

    # Vision Callback
    def vision_callback(self, msg):
        self.vision = msg.data

    # Log Callback
    def log_callback(self, msg):
        self.log = msg.data

    # Individual Blimp Publishers #

    # Publish Motor Commands
    def publish_motor_commands(self):
        # Publish motor_commands value to the ROS topic
        msg = Float64MultiArray()
        msg.data = self.motor_commands
        self.pub_motor_commands.publish(msg)
