### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 2- Package Name
project(vision_utils)
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS cv_bridge image_geometry image_transport pcl_ros rosbag
  # AD
  cvstage  people_msgs)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
find_package(PCL REQUIRED common visualization)
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
add_message_files(
  FILES
  Rect.msg
)
### 5 - Invoke message/service/action generation (generate_messages())
generate_messages()
set(devel_pkg_inc ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
configure_file(cmake/img_path.h.in ${devel_pkg_inc}/img_path.h)
install(FILES ${devel_pkg_inc}/img_path.h DESTINATION ${devel_pkg_inc})
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include third_parties
  CATKIN_DEPENDS ${ROSDEPS}
  DEPENDS
  CFG_EXTRAS img_path.h
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include third_parties
  ${catkin_INCLUDE_DIRS}
  /usr/include/openni /usr/include/nite /usr/include/ni
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
SET(AD_USE_NITE ON CACHE BOOL "")
ADD_SUBDIRECTORY(third_parties)
ADD_SUBDIRECTORY(src)
### 8 - Tests to build (catkin_add_gtest())
ADD_SUBDIRECTORY(test)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
