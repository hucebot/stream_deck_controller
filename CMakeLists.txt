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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})