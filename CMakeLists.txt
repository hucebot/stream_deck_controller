cmake_minimum_required(VERSION 3.0.2)
project(stream_deck_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  src/stream_deck_controller.py
  src/open_dashboard_video.py
  src/open_dashboard_robot.py
  src/open_dashboard_wifi.py
  src/open_dxl_input.py
  src/auxiliar_files/camera_initiator.py
  src/auxiliar_files/home_position.py
  src/auxiliar_files/emergency_stop.py
  src/auxiliar_files/reset_odometry.py
  src/auxiliar_files/speech_initiator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})