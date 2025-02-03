# ROS-JAM
## Overview
This folder contains all code and documentation required to communicate between a microcontroller and a ROS2 script running on Docker. Due to the difficult nature of getting micro-ROS to work on microcontrollers, this folder provides a simpler alternative.

The ROS2 python script and the microcontroller communicate via simple USB serial communication. The ROS2 python script is then responsible for converting between serial messages and ROS2 messages.
