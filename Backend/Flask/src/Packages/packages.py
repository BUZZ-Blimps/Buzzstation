# ========== Packages ========== #

"""
Description:

Top level File for finding packages used in this program.

"""

# Flask Packages
from flask import Flask, render_template, request, Response, jsonify
from flask_socketio import SocketIO, send

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
from time import time # used
import numpy as np # used
import atexit
import sys # used
import signal # used
import os # used
import serial # used
import json # used
import threading # not used
import subprocess # not used
import traceback # not used

# Initialize Flask App and SocketIO
app = Flask('Backend Server')
socketio = SocketIO(app, cors_allowed_origins="*")

# Connect to Redis server
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)
