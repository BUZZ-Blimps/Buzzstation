# ========== Update Blimp Data ========== #

"""
Description:

"""

# Imports
from Packages.packages import redis_client, socketio
from ...Communication.publishers import publish_generic

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Generic Function for updating the color of a ROS topic's UI component for a Blimp's component
def update_blimp_component_color(blimp, component, default_color, nondefault_color):
    if hasattr(blimp, component):

        # Get Redis Value for the Blimp's Component
        current_value = redis_client.hget(f'blimp:{blimp.name}', component)
        
        if current_value is not None:

            # Redis Value
            current_value = current_value.decode('utf-8')
            
            # Blimp Value
            blimp_component_value = getattr(blimp, component)

            if str(blimp_component_value) != str(current_value):

                # Get Color
                if component == 'vision':
                    # Default Value is True
                    color = default_color if blimp_component_value else nondefault_color
                    if color == 'green':
                        # To-Do: Startup Blimp Vision Code
                        logger.info('Restarting Vision for ' + blimp.name)
                    else:
                        # To-Do: Stop Blimp Vision Code
                        logger.info('Stopping Vision for ' + blimp.name)
                else:
                    # Default Value is False
                    color = default_color if not blimp_component_value else nondefault_color

                # Update Frontend
                socketio.emit('update_button_color', {'name': blimp.name, 'key': component, 'color': color})
                
                # Publish if the value has changed
                publish_generic(f'publish_{component}', blimp)
        else:
            
            # Sets Component Color to Default at Start of Program
            socketio.emit('update_button_color', {'name': blimp.name, 'key': component, 'color': default_color})

# Generic Function for updating the color of a ROS topic's UI component for a Blimp's component
def update_blimp_component_value(blimp, component):
    if hasattr(blimp, component):

        # Get Redis Value for the Blimp's Component
        current_value = redis_client.hget(f'blimp:{blimp.name}', component)
        
        if current_value is not None:

            # Redis Value
            current_value = current_value.decode('utf-8')
            
            # Blimp Value
            blimp_component_value = getattr(blimp, component)

            if str(blimp_component_value) != str(current_value):

                # Update Frontend
                socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': current_value})
                
        elif component == 'state_machine':
            
            # Sets Component Value to Default at Start of Program
            socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': 'None'})

        elif component == 'height':
            
            # Sets Component Color and Value to Default at Start of Program
            socketio.emit('update_button_color', {'name': blimp.name, 'key': component, 'color': 'red'})
            socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': 'None'})

# Generic Function for updating the color of a ROS topic's UI component for all blimps (Global Value i.e. Goal Color, Enemy Color)
def update_component_for_all_blimps(basestation_node, component, default_color, nondefault_color):
    current_value = bool(int(redis_client.get(component).decode('utf-8')))
    if getattr(basestation_node, component) is not current_value:

        # Check if list is empty or not (True = Not Empty, False = Empty)
        if not basestation_node.current_blimps:
            # Change Global Component Anyway
            update_global_component_color(basestation_node, component, default_color, nondefault_color)
        else:
            for name, blimp in basestation_node.current_blimps.items():
                if hasattr(blimp, component):
                    # Publish over ROS
                    if getattr(blimp, component) is not current_value:
                        setattr(blimp, component, current_value)
                        publish_generic('publish_' + str(component), blimp)

            setattr(basestation_node, component, current_value)
            
            # Update Frontend
            if not getattr(basestation_node, component):
                socketio.emit('update_button_color', {'name': 'none', 'key': component, 'color': default_color})
            else:
                socketio.emit('update_button_color', {'name': 'none', 'key': component, 'color': nondefault_color})
        
        # Testing
        if str(component) == 'goal_color':
            logger.info(str('Overall Goal Color: ' + str(basestation_node.goal_color)))
        elif str(component) == 'enemy_color':
            logger.info(str('Overall Enemy Color: ' + str(basestation_node.enemy_color)))

# Generic Function for updating the color of a ROS topic's UI component when no Blimps exist
def update_global_component_color(basestation_node, component, default_color, nondefault_color):
    current_component_color = bool(int(redis_client.get(component).decode('utf-8')))
    if getattr(basestation_node, component) is not current_component_color:
        setattr(basestation_node, component, current_component_color)

        # Update Frontend
        if not getattr(basestation_node, component):
            socketio.emit('update_button_color', {'name': 'none', 'key': component, 'color': default_color})
        else:
            socketio.emit('update_button_color', {'name': 'none', 'key': component, 'color': nondefault_color})

        # Testing
        if str(component) == 'goal_color':
            logger.info(str('Overall Goal Color: ' + str(basestation_node.goal_color)))
        elif str(component) == 'enemy_color':
            logger.info(str('Overall Enemy Color: ' + str(basestation_node.enemy_color)))
