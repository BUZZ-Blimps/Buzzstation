# Buzzstation

## Description 
A React Native app to send commands and receive data to/from blimps using ROS 2

**Note:** This app is not yet complete. It is still in the developmental stage and has only been tested with fake blimps.

![Buzz_Blimps_App](https://github.com/awilwayco/Buzzstation/assets/56363833/7b55024e-7c6e-4cfa-b207-a733a153444a)
<p align="center">
<em>Figure 1. Basestation UI on a Mobile Device</em>
  
<em>Multi-Controller Functionality: The green blimp names indicate blimps not under control, blue indicates the blimp the device currently controls, and red indicates blimps being controlled by other devices. Each device can only control a single blimp at a time. To unselect a blimp, press the blue button to change it back to green. </em>
</p>
</p>

## Requirements

- A device with the Ubuntu 22.04 Operating System (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

## To-Do

### Backend

Features
- Backend virtual joystick usage
- Re-add rosjoy functionality and test

### Frontend

Features
- Add Auto Button
- Add virtual joysticks for the app and test performance/functionality when multiple blimps are being controlled
- Add State Machine text (with dropdown)
- Add All Auto and All Manual Buttons
- Add Calibrate Button (with Height value)
- Column Headers
- Fix Spacing on all platforms/devices (Make buttons resize based on screen dimensions)
- Barometer Integration
- Tuning Page
- Vision ON/OFF Buttons
- Sidebar menu
