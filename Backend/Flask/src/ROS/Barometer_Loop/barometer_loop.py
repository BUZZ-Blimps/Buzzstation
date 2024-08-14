# ========== Barameter Loop ========== #

"""
Description:

Connects to the Barometer via a Serial Port and
send Barometer data over ROS.

"""

from Packages.packages import serial, socketio
from ..ros import basestation_node, redis_client
from ..Communication.publishers import publish_barometer

# Barometer Timer Loop
def barometer_loop():

    # Barometer Serial Port Not Connected
    if basestation_node.barometer_serial == None:

        try:

            basestation_node.barometer_serial = serial.Serial('/dev/ttyACM0', 115200)
            basestation_node.get_logger().info('BAROMETER CONNECTED')

            # Emit Barometer Button Color to Frontend
            socketio.emit('update_barometer_button_color', 'green')

        except Exception as e:

            # Testing
            #basestation_node.get_logger().error(str(e))

            try:

                basestation_node.barometer_serial = serial.Serial('/dev/ttyACM1', 115200)
                basestation_node.get_logger().info('BAROMETER CONNECTED')

            except Exception as e:

                # Testing
                #basestation_node.get_logger().error(str(e))

                # Testing
                #basestation_node.get_logger().info('BAROMETER NOT CONNECTED')

                # Emit Barometer Button Color to Frontend
                socketio.emit('update_barometer_button_color', 'red')

                # Emit Barometer Reading to Frontend
                socketio.emit('barometer_reading', 'Disconnected')

    # Barometer Serial Port Connected
    else:

        # Read Barometer Data if Available
        if basestation_node.barometer_serial.in_waiting:

            # Read a line of data from the serial port
            basestation_node.barometer_reading = float(basestation_node.barometer_serial.readline().decode('utf-8'))
            basestation_node.barometer_serial.flushInput()

            # Add Barometer Value to Redis
            redis_client.set('barometer', basestation_node.barometer_reading)

            # Emit Barometer Reading to Frontend
            socketio.emit('barometer_reading', basestation_node.barometer_reading)

            # Publish to ROS
            publish_barometer(basestation_node)

        else:
            socketio.emit('update_barometer_button_color', 'red')

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
