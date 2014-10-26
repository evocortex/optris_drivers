cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
rosbuild_gensrv()

INCLUDE_DIRECTORIES(/usr/include)
LINK_DIRECTORIES(/usr/lib)
LINK_LIBRARIES(irimager irimageprocessing udev)

rosbuild_add_executable(optris_imager_node src/optris_imager_node.cpp)
rosbuild_add_executable(optris_colorconvert_node src/optris_colorconvert_node.cpp)
