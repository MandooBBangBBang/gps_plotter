cmake_minimum_required(VERSION 3.0.2)
project(gps_plotter)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(

)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gps_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

