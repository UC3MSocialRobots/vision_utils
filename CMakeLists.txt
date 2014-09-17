cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

### include the macros
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake" )
include(CMakeMacros)

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/img_path.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/vision_utils/img_path.h"
)


### include the build path for the generated headers
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/build)

### build the custom messages and services
rosbuild_genmsg()
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/msg_gen/cpp/include)
#~ rosbuild_gensrv()
#~ INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/srv_gen/cpp/include)

### deps
### tesseract
AD_CHECK_PROGRAM_INSTALLED(tesseract)

#FIND_PACKAGE( OpenCV REQUIRED NO_MODULE)
FIND_PACKAGE(OpenCV REQUIRED)
#~ MESSAGE(STATUS "OpenCV_LIBS:${OpenCV_LIBS} - OpenCV_DIR:${OpenCV_DIR}")
find_package(PCL 1.2 REQUIRED)

### third parties
AD_CHECK_LIBRARY_INSTALLED(OpenNI XnVNite)
INCLUDE_DIRECTORIES("/usr/include/ni")
INCLUDE_DIRECTORIES("/usr/include/nite")
INCLUDE_DIRECTORIES("/usr/include/openni")
ADD_SUBDIRECTORY(third_parties)

### compile !
SET(AD_USE_ROS 1) # needed for CMakeMacros to use ROS building routines
SET(AD_USE_OPENCV 1 CACHE BOOL "")
SET(AD_USE_NITE 1 CACHE BOOL "")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
ADD_SUBDIRECTORY(vision_utils)
ADD_SUBDIRECTORY(test)
