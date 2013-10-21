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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

set(mesa_element_SRCS
  include/tedusar_mesa_element/mesa_element.h
  include/tedusar_mesa_element/mesa_element_config.h
  include/tedusar_mesa_element/mesa_element_state.h
  src/mesa_element.cpp
)

rosbuild_add_library(mesa_element ${mesa_element_SRCS})
rosbuild_link_boost(mesa_element system thread regex)

set(mesa_element_node_SRCS
  include/tedusar_mesa_element/mesa_element_node.h
  src/mesa_element_node.cpp
)

rosbuild_add_executable(mesa_element_node ${mesa_element_node_SRCS})
target_link_libraries(mesa_element_node mesa_element)
