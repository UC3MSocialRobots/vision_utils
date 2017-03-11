cmake_minimum_required(VERSION 2.8.3)
project(vision_utils)
find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS  include
)

# make all headers visible
# http://stackoverflow.com/questions/28384935/qtcreator-cmake-project-how-to-show-all-project-files
FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(qtcreator_headers SOURCES ${LibFiles})
