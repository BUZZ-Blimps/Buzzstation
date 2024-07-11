# Buzzstation

## Description 
A React Native app to send commands and receive data to/from blimps using ROS 2

**Note:** This app is not yet complete. It is still in the developmental stage and has only been tested with fake blimps.

<p align="center">
<img src=https://github.com/SWAMP-Blimps/Buzzstation/assets/56363833/892e5403-dc56-44fd-898b-d7f492281e90) />
</p>
<p align="center">
<em>Figure 1. Basestation UI</em>
</p>
<p align="center">
<em>Multi-Controller Functionality: The green blimp names indicate blimps not under control, blue indicates the blimp the device currently controls, and red indicates blimps being controlled by other devices. Each device can only control a single blimp at a time. To unselect a blimp, press the blue button to change it back to green. </em>
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
- Add Sidebar Menu
- Add Logs Page
- Add Tuning Page
- Dropdown Menu for State Machine
- Select and deselect visible blimps on Main Page
- Add Virtual Joysticks for the app and test performance and functionality when multiple blimps are being controlled
- Add Vision ON/OFF Buttons

Bugs
- Fix Spacing on all platforms/devices (Make buttons resize based on screen dimensions)
- Fix Multi-controller issue of when a device or browser closes and the blimp name button stays red (Add a timeout to check for the device)

## Ideas

### Schematics

<p align="center">
<img src=https://github.com/SWAMP-Blimps/Buzzstation/assets/56363833/e6a9dfd9-489b-472e-a24c-880bda73f7cc) />
</p>
<p align="center">
<em>Figure 2. Main Page Schematic</em>
</p>

<p align="center">
<img src=https://github.com/awilwayco/Buzzstation/assets/56363833/b473b9cc-6c5c-47ab-b007-11b4d6503f2f) />
</p>
<p align="center">
<em>Figure 3. Tuning Page Schematic</em>
</p>

<p align="center">
<img src=https://github.com/awilwayco/Buzzstation/assets/56363833/43eaec0d-e121-4890-a705-e21683139139) />
</p>
<p align="center">
<em>Figure 3. Tuning Page Schematic</em>
</p>

### Suggestions
- Use a steam deck or mobile device to control blimps instead of an Xbox controller
