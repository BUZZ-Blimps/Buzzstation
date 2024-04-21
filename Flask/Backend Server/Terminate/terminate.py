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

    # Destroy Basestation Node
    print('\nDestroying Basestation Node...')
    global node
    try:
        node.destroy_node()
    except rclpy.handle.InvalidHandle as e:
        pass

    # Shutdown Rclpy
    try:
        rclpy.shutdown()
    except Exception as e:
        pass

    # Close Barometer Serial Port
    for blimp in blimps:
        try:
            blimps[blimp].parent_node.barometer.close()
        except:
            pass

    # Kill PID
    print('\nTerminating Program...\n')
    try:
        os.kill(current_pid, sig.SIGTERM)
    except AttributeError:
        os.kill(current_pid, signal.SIGTERM)
    
    # If all else fails...
    subprocess("kill -9 " + str(current_pid))
