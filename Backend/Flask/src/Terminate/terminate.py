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
    print("\nPerforming cleanup...\n")

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
        print('Basestation Node Destroyed.\n')
    except rclpy.handle.InvalidHandle as e:
        print('Error: Failed Destroying Basestation Node\n')

    # Shutdown Rclpy
    try:
        if rclpy.ok():
            rclpy.shutdown()
        print('Rclpy Shutdown.\n')
    except Exception as e:
        print('Error: Failed Shutting Down Rclpy\n')

    # Close Barometer Serial Port
    try:
        basestation_node.barometer_serial.close()
        print('Barometer Serial Port Closed.\n')
    except:
        print('No Barometer Serial Port Found.\n')

    # Finished Cleanup
    print("Cleanup complete.\n")

    # Start Program Termination
    print('Terminating Program...\n')

# Terminate Code
def terminate(sig, frame):

    # Kill the Main Script
    pid = find_script_by_env_var('UNIQUE_ID', 'buzzstation')
    if pid:
        os.kill(int(pid), signal.SIGINT)

    # Kill PID
    print('Terminating Program...\n')
    try:
        os.kill(current_pid, sig.SIGTERM)
    except AttributeError:
        os.kill(current_pid, signal.SIGTERM)
    
    # If all else fails...
    subprocess("kill -9 " + str(current_pid))

    print('Failed Termination.\n')

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
                    #print(f'\nScript found with PID: {pid}')
                    return pid
            except Exception as e:
                pass

        # Testing
        #print('Script not found')
        return None
    except Exception as e:
        print(f'Error: {e}')
