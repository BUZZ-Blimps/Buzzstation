# Buzzstation

## Description 
A React Native app to send commands and receive data to/from blimps using ROS 2

**Note:** This app is not yet complete. It is still in the developmental stage and has only been tested with fake blimps.
<p align="center">
  
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

Necessary Features
- Add Full Controller Functionality for web and app users
- Add State Machine text
- Add 'All Auto' and 'All Manual' Buttons
- Add Calibrate Button (with Height value)
- Add Column Headers
- Add Barometer Integration

"Nice to Have" Features
- Add Sidebar menu
- Add Logs Page
- Add Tuning Page
- Dropdown menu for State Machine
- Select and deselect visible blimps on Main Page
- Add Virtual Joysticks for the app and test performance and functionality when multiple blimps are being controlled
- Add Vision ON/OFF Buttons

Bugs
- Fix Spacing on all platforms/devices (Make buttons resize based on screen dimensions)
- Fix Multi-controller issue of when a device or browser closes and the blimp name button stays red (Add a timeout to check for the device)

## Ideas

### Schematics

<p align="center">

![Future Basestation Idea Schematic](https://github.com/awilwayco/Buzzstation/assets/56363833/42d1ada5-12ba-4d33-97d4-c54773996f03)
<p align="center">
<em>Figure 2. Main Page Schematic</em>
</p>
</p>

<p align="center">

![Tuning Page Idea](https://github.com/awilwayco/Buzzstation/assets/56363833/b473b9cc-6c5c-47ab-b007-11b4d6503f2f)
<p align="center">
<em>Figure 3. Tuning Page Schematic</em>
</p>
</p>

<p align="center">

![Controller Mapping Idea](https://github.com/awilwayco/Buzzstation/assets/56363833/43eaec0d-e121-4890-a705-e21683139139)
<p align="center">
<em>Figure 3. Tuning Page Schematic</em>
</p>
</p>

### Suggestions
- Use a steam deck or mobile device to control blimps instead of an Xbox controller
