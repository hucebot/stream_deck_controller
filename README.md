# stream_deck_controller

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros1 Version](https://img.shields.io/badge/ROS-Noetic-green)](
http://wiki.ros.org/noetic)

Package to control the Elgato Stream Deck using ROS. This package it's oriented to control robots with the cartesian interface in a more intuitive way.

# Get Started

## Installation
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command in the docker folder:

## Usage

```bash
# Command 1
mkdir -p /home/forest_ws/build/stream_deck_controller
cd /home/forest_ws/build/stream_deck_controller
```
```bash
# Command 2
source /opt/ros/noetic/setup.bash && source /home/forest_ws/setup.bash && cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/stream_deck_controller && make -j8 && make install
```
Then in the file /home/forest_ws/setup.bash copy the following line:
```bash
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/home/forest_ws/src/tiago_dual_cartesio_config:/home/forest_ws/src/tiago_dual_robot:/home/forest_ws/src/tiago_dual_description_calibration:/home/forest_ws/src/pal_urdf_utils:/home/forest_ws/src/omni_base_robot:/home/forest_ws/src/tiago_robot:/home/forest_ws/src/hey5_description:/home/forest_ws/src/pmb2_robot:/home/forest_ws/src/pal_gripper:/home/forest_ws/src/stream_deck_controller"
```
For each change in the package you need to compile the package again (just the second command) and then source the setup.bash file from the workspace.
