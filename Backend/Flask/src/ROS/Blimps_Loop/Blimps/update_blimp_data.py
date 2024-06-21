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
    # To-Do: Fix Redis Access for a Specific Blimp Component
    current_component_color = bool(int(redis_client.get(component).decode('utf-8')))
    if getattr(blimp, component) is not current_component_color:
        setattr(blimp, component, current_component_color)

        # Update Frontend
        if not getattr(blimp, component):
            socketio.emit('update_button_color', {'blimp': blimp.name, 'key': component, 'color': default_color})
        else:
            socketio.emit('update_button_color', {'blimp': blimp.name, 'key': component, 'color': nondefault_color})

        # Publish over ROS
        # Call the publish method for the specified component over ROS
        publish_generic('publish_' + str(component), blimp)

# Generic Function for updating the color of a ROS topic's UI component for all blimps (Global Value i.e. Goal Color, Enemy Color, All Auto State)
def update_component_for_all_blimps(basestation_node, component, default_color, nondefault_color):
    current_value = bool(int(redis_client.get(component).decode('utf-8')))
    if getattr(basestation_node, component) is not current_value:

        # Check if list is empty or not (True = Not Empty, False = Empty)
        if not basestation_node.current_blimps:
            # Change Global Component Anyway
            update_global_component_color(basestation_node, component, default_color, nondefault_color)
        else:
            for blimp_name, blimp in basestation_node.current_blimps.items():
                if hasattr(blimp, component):
                    # Publish over ROS
                    if getattr(blimp, component) is not current_value:
                        setattr(blimp, component, current_value)
                        publish_generic('publish_' + str(component), blimp)

            setattr(basestation_node, component, current_value)
            
            # Update Frontend
            if not getattr(basestation_node, component):
                socketio.emit('update_button_color', {'blimp': 'none', 'key': component, 'color': default_color})
            else:
                socketio.emit('update_button_color', {'blimp': 'none', 'key': component, 'color': nondefault_color})
        
        # Testing
        if str(component) == 'goal_color':
            logger.info(str('Overall Goal Color: ' + str(basestation_node.goal_color)))
        elif str(component) == 'enemy_color':
            logger.info(str('Overall Enemy Color: ' + str(basestation_node.enemy_color)))
        elif str(component) == 'all_auto_state':
            logger.info(str('All Auto State: ' + str(basestation_node.all_auto_state)))

# Generic Function for updating the color of a ROS topic's UI component when no Blimps exist
def update_global_component_color(basestation_node, component, default_color, nondefault_color):
    current_component_color = bool(int(redis_client.get(component).decode('utf-8')))
    if getattr(basestation_node, component) is not current_component_color:
        setattr(basestation_node, component, current_component_color)

        # Update Frontend
        if not getattr(basestation_node, component):
            socketio.emit('update_button_color', {'blimp': 'none', 'key': component, 'color': default_color})
        else:
            socketio.emit('update_button_color', {'blimp': 'none', 'key': component, 'color': nondefault_color})
