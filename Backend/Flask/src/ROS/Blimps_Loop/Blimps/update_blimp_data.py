# ========== Update Blimp Data ========== #

"""
Description:

Update component colors or values for an individual blimp or all blimps.

"""

# Imports
from Packages.packages import redis_client, socketio
from ...Communication.publishers import publish_generic

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Individual Blimp #

# Generic Function for updating the color of a ROS topic's UI component for a Blimp's component
def update_blimp_component_color(blimp, component, default_color, nondefault_color):

    blimp_component_set = False
    if hasattr(blimp, component):
        # Blimp Value
        blimp_component_value = getattr(blimp, component)
        if blimp_component_value is not None:
            blimp_component_set = True

    if blimp_component_set:

        # Get Redis Value for the Blimp's Component
        redis_value = redis_client.hget(f'blimp:{blimp.name}', component)
        
        update_value = False
        if redis_value is None:
            update_value = True
        else:
            redis_value = redis_value.decode('utf-8')
            if str(redis_value) != str(blimp_component_value):
                update_value = True

        if update_value:

            # logger.info("update value of " + str(component) + " to " + str(blimp_component_value))

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
            color_update_val = {'name': blimp.name, 'key': component, 'color': color}
            # logger.info(str(color_update_val))
            socketio.emit('update_button_color', color_update_val)
            
            # Update redis
            redis_client.hset(f'blimp:{blimp.name}', component, str(blimp_component_value))

            # Publish if the value has changed
            publish_generic(f'publish_{component}', blimp)

    else:
        redis_client.hdel(f'blimp:{blimp.name}', component)

        # Sets Component Color to Default at Start of Program
        socketio.emit('update_button_color', {'name': blimp.name, 'key': component, 'color': default_color})

# Generic Function for updating the value of a ROS topic's UI component for a Blimp's component
def update_blimp_component_value(blimp, component):

    blimp_component_set = False
    if hasattr(blimp, component):
        # Blimp Value
        blimp_component_value = getattr(blimp, component)
        if blimp_component_value is not None:
            blimp_component_set = True

    if blimp_component_set:

        # Get Redis Value for the Blimp's Component
        redis_value = redis_client.hget(f'blimp:{blimp.name}', component)

        update_value = False
        if redis_value is None:
            update_value = True
        else:
            redis_value = redis_value.decode('utf-8')
            if str(redis_value) != str(blimp_component_value):
                update_value = True

        if update_value:
            # logger.info("update value of " + str(component) + " to " + str(blimp_component_value))

            # Update redis
            redis_client.hset(f'blimp:{blimp.name}', component, str(blimp_component_value))

            # Update Frontend
            socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': blimp_component_value})

            # Publish if the value has changed
            publish_generic(f'publish_{component}', blimp)
    else:
        redis_client.hdel(f'blimp:{blimp.name}', component)

        if component == 'state_machine':
            # Sets Component Value to Default at Start of Program
            socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': 'None'})

        elif component == 'height':
            # Sets Component Color and Value to Default at Start of Program
            socketio.emit('update_button_color', {'name': blimp.name, 'key': component, 'color': 'red'})
            socketio.emit('update_button_value', {'name': blimp.name, 'key': component, 'value': 'None'})

# All Blimps #

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
