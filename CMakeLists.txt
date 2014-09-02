cmake_minimum_required(VERSION 2.4.6)

find_package(catkin QUIET COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

IF(catkin_FOUND)
	project(optris_drivers)
	## Generate services in the 'srv' folder
	add_service_files(
	   FILES
	   AutoFlag.srv
	   ForceFlag.srv
	 )

	## Generate added messages and services with any dependencies listed here
	generate_messages()

	###################################
	## catkin specific configuration ##
	###################################
	catkin_package(
	  INCLUDE_DIRS include
	  CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs std_srvs
	  DEPENDS ImageProcessing udev
	)

	###########
	## Build ##
	###########
	include_directories(
	  ${catkin_INCLUDE_DIRS}
	  ${PROJECT_SOURCE_DIR}/include
	)

	link_directories(${PROJECT_SOURCE_DIR}/lib/i386 ${PROJECT_SOURCE_DIR}/lib/x86_64)
	link_libraries(${catkin_LIBRARIES} PIImager ImageProcessing udev pthread)

	add_executable(optris_imager_node src/optris_imager_node.cpp)
	add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp)
ELSE()
	include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

	# Set the build type.  Options are:
	#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
	#  Debug          : w/ debug symbols, w/o optimization
	#  Release        : w/o debug symbols, w/ optimization
	#  RelWithDebInfo : w/ debug symbols, w/ optimization
	#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
	#set(ROS_BUILD_TYPE RelWithDebInfo)

	rosbuild_init()

	#set the default path for built executables to the "bin" directory
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
	#set the default path for built libraries to the "lib" directory
	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

	#uncomment if you have defined messages
	#rosbuild_genmsg()
	#uncomment if you have defined services
	rosbuild_gensrv()

	#common commands for building c++ executables and libraries
	#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
	#target_link_libraries(${PROJECT_NAME} another_library)
	#rosbuild_add_boost_directories()
	#rosbuild_link_boost(${PROJECT_NAME} thread)
	#rosbuild_add_executable(example examples/example.cpp)
	#target_link_libraries(example ${PROJECT_NAME})

	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
	LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib/i386 ${PROJECT_SOURCE_DIR}/lib/x86_64)
	LINK_LIBRARIES(PIImager ImageProcessing udev)

	rosbuild_add_executable(optris_imager_node src/optris_imager_node.cpp)
	rosbuild_add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp)
ENDIF()
