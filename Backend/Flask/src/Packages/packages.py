# ========== Packages ========== #

"""
Description:

Top level File for finding packages used in this program.

To-Do:

- Separate into separate files

"""

# Flask Packages
from flask import Flask, render_template, request, Response, jsonify
from flask_socketio import SocketIO, send
from flask_cors import CORS

# Redis Package
import redis

# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray
try:
    from yolo_msgs.msg import BoundingBox # type: ignore
except:
    pass
from sensor_msgs.msg import Joy
import launch.actions

# Livestream Packages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Other Packages
import threading
import time
import sys
import signal
import os
import serial
import numpy as np
import subprocess
import json # Currently not used (Potential future use)
import traceback

# Initialize Flask App and SocketIO
app = Flask('Backend Server')
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Connect to Redis server
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

# Blimps
global blimps
blimps = {}