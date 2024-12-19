# stream_deck_controller

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros1 Version](https://img.shields.io/badge/ROS-Noetic-green)](
http://wiki.ros.org/noetic)

Package to control the Elgato Stream Deck using ROS. This package it's oriented to control robots with the cartesian interface in a more intuitive way.

# Get Started

## Installation
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command in the docker folder:

### From Docker (Dev Mode)

```bash
docker build -t stream_deck -f .ci/Dockerfile 
```

# Usage

Once you have the docker image, you can run the container with the following command :
## Dev Mode

You can run the container with the following command:
```bash
bash .ci/run_dev_docker.sh
```
Once is runnig, because the docker image is running with also the cartesIO image you need to compile the stream deck controller package with the following commands:
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

## Deployment Mode
```bash
bash .ci/run_deployment_docker.sh
```
This will start the container and open a shell. To start the stream deck controller, run the following command:

```roslaunch stream_deck_controller streamdeck_controller.launch```

This will start 4 nodes:
- `streamdeck_controller`: The main node that controls the stream deck.
- `ros_cartesio_initiator`: A node that listens to the stream deck and enable or disable the cartesian interface.
- `ros_bridge_initiator`: A node that listens to the stream deck and enable or disable the ros bridge control.
- `ros_camera_initiator`: A node that listens to the stream deck and enable or disable the camera information.

Also the package includes to launch the different robot dashboard views that are included in the package: [robot_dashboard](https://github.com/hucebot/robot_dashboard).