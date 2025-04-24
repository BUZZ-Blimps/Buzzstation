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
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray, Int64MultiArray, Float32MultiArray
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
import ast # used
import atexit # used
import sys # used
import signal # used
import os # used
import serial # used
import json # used
import yaml # used
import threading # not used
import subprocess # not used
import traceback # not used

# Initialize Flask App and SocketIO
app = Flask('Backend Server')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Connect to Redis server
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

# Logger
from rclpy.logging import get_logger
logger = get_logger('Basestation')

# Load allowed Wi-Fi networks from the YAML file
def load_allowed_wifi_networks(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data.get('allowed_wifi_networks', [])

# Check if the current Wi-Fi Network is allowed
def check_wifi():
    # Load allowed Wi-Fi networks
    allowed_networks = load_allowed_wifi_networks('../src/Config/wifi_networks.yaml')
    
    # Get current SSID
    output = subprocess.check_output(["iwconfig 2>/dev/null | grep 'ESSID:' | cut -d '\"' -f 2"], shell=True)
    ssid = output.decode('utf-8').strip()
    
    # Check if the current SSID is in the allowed list
    if ssid not in allowed_networks:
        logger.info("Invalid WiFi selected! Must be on one of the allowed networks.")
        logger.info(f'The Current SSID is: $$${ssid}$$$')
        return False
    else:
        return True
