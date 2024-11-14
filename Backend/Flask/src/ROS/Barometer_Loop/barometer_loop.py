# ========== Barameter Loop ========== #

"""
Description:

Connects to the Barometer via a ROS

"""

from Packages.packages import serial, socketio, time
from ..ros import basestation_node, redis_client
from ..Communication.publishers import publish_barometer

# Barometer Timer Loop
def barometer_loop():
    if basestation_node.barometer_online is True and time()-basestation_node.barometer_time > 5.0:
        socketio.emit('update_barometer_button_color', 'red')
        socketio.emit('barometer_reading', 'Disconnected')
        basestation_node.barometer_online = False

    elif basestation_node.barometer_online is True:
        basestation_node.redis_client.set('barometer', round(basestation_node.barometer_reading,2))
        socketio.emit('update_barometer_button_color', 'green')
        socketio.emit('barometer_reading', round(basestation_node.barometer_reading,2))

    else:
        socketio.emit('update_barometer_button_color', 'red')
        socketio.emit('barometer_reading', 'Disconnected')    

# Fake Barometer Timer Loop
def fake_barometer_loop():

    # Emit Barometer Button Color to Frontend
    socketio.emit('update_barometer_button_color', 'green')

    # Set barometer value
    #import random
    #basestation_node.barometer_reading = round(random.uniform(99000.00, 110000.00), 2)

    # Add Barometer Value to Redis
    redis_client.set('barometer', basestation_node.barometer_reading)

    # Emit Barometer Reading to Frontend
    socketio.emit('barometer_reading', basestation_node.barometer_reading)

    # Publish to ROS
    publish_barometer(basestation_node)
