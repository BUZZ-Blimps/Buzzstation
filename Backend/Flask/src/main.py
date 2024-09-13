#!/usr/bin/env python3

# ========== Main ========== #

"""
Program Description:

This program acts as the Backend Server to the Basestation.
ROS and SocketIO are used in this program as communication
tools. Redis is also used as a caching database for data.

ROS: All ROS Communication occurs through the device this 
program is running on. 

SocketIO: This program communicates with the Frontend Server
using SocketIO which is a libary that uses Websockets to 
send and recieve JSON data.

Redis: This program uses a caching database called Redis,
which is used to store data with sub-millisecond speed 
and prevent read/write errors between SocketIO and ROS.

"""

"""
File Description:

- Creates a signal where if CTRL+C is pressed, program is terminated
- Starts ROS 2 Thread (ROS Communication for Basestation)
- Starts Flask Server (Backend Server for App)

"""

# Imports
from Packages.packages import check_wifi
from Terminate.terminate import terminate, cleanup, signal, sys, os, atexit
from SocketIO.Senders.redis import init_redis_values
from ROS.ros import start_ros
from SocketIO.socketio import start_backend_server

if __name__ == '__main__':

    # Register cleanup function
    atexit.register(cleanup)

    # Terminate if Ctrl+C Caught
    signal.signal(signal.SIGINT, terminate)

    if check_wifi():

        # Init Redis Values
        init_redis_values()

        # Start ROS 2
        start_ros()

        # Start Backend Server with current device IP on Port 5000
        start_backend_server(sys.argv[1], 5000)

    else:

        # Simulate sending the SIGINT signal to the current process
        os.kill(os.getpid(), signal.SIGINT)
