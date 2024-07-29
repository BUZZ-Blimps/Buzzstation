# ========== Terminate ========== #

"""
Description:

- Destroys the Basestation Node
- Shuts down Rclpy (Python ROS Thread in use)
- Closes Barometer Serial Port
- Kills Process ID (Program Terminates)

"""

# Imports
from Packages.packages import *

# Get the PID of the current process
current_pid = os.getpid()

# Terminate Code
def terminate(sig, frame):
    from ROS.ros import basestation_node

    # Destroy Basestation Node
    print('\nDestroying Basestation Node...\n')
    try:
        basestation_node.destroy_node()
    except rclpy.handle.InvalidHandle as e:
        print('\nError: Failed Destroying Basestation Node\n')

    # Shutdown Rclpy
    print('Shutting down Rclpy...\n')
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print('\nError: Failed Shutting Down Rclpy\n')

    # Kill the Main Script
    pid = find_script_by_env_var('UNIQUE_ID', 'buzzstation')
    if pid:
        os.kill(int(pid), signal.SIGINT)

    # Emit to Frontend
    socketio.emit('update_names', '')

    # Kill PID
    print('Terminating Program...\n')
    try:
        os.kill(current_pid, sig.SIGTERM)
    except AttributeError:
        os.kill(current_pid, signal.SIGTERM)
    
    # If all else fails...
    subprocess("kill -9 " + str(current_pid))

    print('Failed Termination.\n')

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
