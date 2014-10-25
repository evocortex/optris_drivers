cmake_minimum_required(VERSION 2.8.3)
project(optris_drivers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  image_transport
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    AutoFlag.srv
    ForceFlag.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_srvs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
# LIBRARIES
 CATKIN_DEPENDS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  image_transport
  message_runtime
 DEPENDS
  libudev-dev
  Boost
)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(optris_imager_node src/optris_imager_node.cpp)
add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp)

## make sure msg/srvs are generated before building the targets
## also, the libs must be copied before trying to build the other files
add_dependencies(optris_imager_node irimager irimageprocessing)
add_dependencies(optris_colorconvert_node irimager irimageprocessing)


## Specify libraries to link a library or executable target against
target_link_libraries(optris_imager_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  irimager
  irimageprocessing
  udev
)

target_link_libraries(optris_colorconvert_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  irimager
  irimageprocessing
  udev
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS optris_colorconvert_node optris_imager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

