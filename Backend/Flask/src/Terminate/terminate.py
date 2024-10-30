# ========== Terminate ========== #

"""
Description:

- Sets Redis Values to Default
- Updates All User UI's with No Blimps
- Destroys the Basestation Node
- Shuts down Rclpy (Python ROS Thread in use)
- Closes Barometer Serial Port
- Kills Process ID (Program Terminates)

"""

# Imports
from Packages.packages import *

# Get the PID of the current process
current_pid = os.getpid()

# Cleanup Code before Termination
def cleanup():

    # Starting Cleanup
    logger.info("Performing cleanup...\n")

    from SocketIO.Senders.redis import init_redis_values, get_redis_values

    # Deselect from any connected blimps on Frontend UI
    current_names = redis_client.get('current_names').decode("utf-8")
    for name in current_names.split(','):
        # Make Blimp Button Green for all users
        socketio.emit('toggle_name_button_color', { 'userID': 'none', 'name': name})

    # Set Redis Values back to Default
    init_redis_values()

    # Send Default Redis Values to Frontend
    get_redis_values()

    # Emit No Blimps to Frontend
    socketio.emit('update_names', '')

    from ROS.ros import basestation_node

    # Destroy Basestation Node
    try:
        basestation_node.destroy_node()
        logger.info('Basestation Node Destroyed.\n')
    except rclpy.handle.InvalidHandle as e:
        logger.info('Error: Failed Destroying Basestation Node\n')

    # Shutdown Rclpy
    try:
        if rclpy.ok():
            rclpy.shutdown()
        logger.info('Rclpy Shutdown.\n')
    except Exception as e:
        logger.info('Error: Failed Shutting Down Rclpy\n')

    # Close Barometer Serial Port
    try:
        basestation_node.barometer_serial.close()
        logger.info('Barometer Serial Port Closed.\n')
    except:
        logger.info('No Barometer Serial Port Found.\n')

    # Finished Cleanup
    logger.info("Cleanup complete.\n")

    # Start Program Termination
    logger.info('Terminating Program...\n')

# Terminate Code
def terminate(sig, frame):

    # Kill the Main Script
    pid = find_script_by_env_var('UNIQUE_ID', 'buzzstation')
    if pid:
        cleanup()
        os.kill(int(pid), signal.SIGTERM)

    # Kill Current PID if Cannot Terminate Main Script
    logger.info('Failed Initial Termination.\n')
    try:
        os.kill(current_pid, sig.SIGTERM)
    except:
        os.kill(current_pid, signal.SIGTERM)
    
    # If all else fails...
    logger.info('Failed Secondary Termination.\n')
    subprocess.run(["kill", "-9", str(current_pid)])

    # Error has occurred
    logger.info('Failed Termination.\n')

# Find script with Environment Variable Name and Value
def find_script_by_env_var(env_var_name, env_var_value):
    try:
        result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE)
        processes = result.stdout.decode().split('\n')
        
        for process in processes:
            pid = process.split()[1]
            try:
                env_vars = open(f'/proc/{pid}/environ').read()
                if env_var_name in env_vars and env_var_value in env_vars:
                    # Testing
                    #logger.info(f'\nScript found with PID: {pid}')
                    return pid
            except Exception as e:
                pass

        # Testing
        #print('Script not found')
        return None
    except Exception as e:
        logger.info(f'Error: {e}')
