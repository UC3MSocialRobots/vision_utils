#!/bin/bash
# this script will make a standalone package for files
# height_pplm.* and color_face_detector.*
# As such, they can be compiled without other AD dependencies.

# load ROS stuff for being able to use "roscp"
PREV_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH         # store value of ROS_PACKAGE_PATH
source /opt/ros/`rosversion -d`/setup.bash      # load roscp, roscd, etc.
export ROS_PACKAGE_PATH=$PREV_ROS_PACKAGE_PATH # restore value of ROS_PACKAGE_PATH

# output folder
PROJECT=height_pplm
PKGDIR=~/mystacks/$PROJECT
SRCDIR=$PKGDIR/src

################################################################################
################################################################################

shorten_include() { # 1:file
  #~ sed -i "s,#include.*[/<]$1.,#include \"$1\",g" $SRCDIR/*.*
  find $SRCDIR  \( -name "*.cpp" -o -name "*.h" \) -exec    \
      sed -i "s,#include.*[/<]$1.,#include \"$1\",g" {} \;
}

add_dep() { # 1:package, 2:file
  ret=`roscp $1 $2 $SRCDIR`
  if [[ $ret != "" ]]
  then
    echo "Failed to copy file '$2' from package '$1': $ret"
    exit
  fi
  shorten_include $2
}

remove_include() { # 1:file
  #~ sed -i "s,#include .*[\"</].*$1[\">],,g" $SRCDIR/*.*
  find $SRCDIR  \( -name "*.cpp" -o -name "*.h" \) -exec    \
    sed -i "s,#include .*[\"</].*$1[\">],,g" {} \;
}


################################################################################
################################################################################

echo ; echo "Creating folder $PKGDIR..."
# go to parent folder
mkdir --parents $PKGDIR ; cd $PKGDIR/.. ; rm $PKGDIR -rf
roscreate-pkg $PROJECT image_transport cv_bridge rosbag image_geometry kdl tf      >> /dev/null
rospack profile >> /dev/null
roscd $PROJECT
# pwd ; exit

echo ; echo "Adding classes..."
mkdir --parents $SRCDIR
add_dep people_recognition_vision height_pplm.cpp
add_dep people_recognition_vision height_pplm.h
add_dep people_recognition_vision height_detector.h
add_dep kinect nite_primitive_standalone.cpp
add_dep kinect nite_primitive.h
add_dep kinect publish_kinect_serial.cpp

# deps
echo ; echo "Adding deps and changing include paths..."
# from most specifici to most generic: a file should only depend on the following ones
add_dep  people_msgs  pplm_template.h
add_dep  vision_utils  nano_skill.h
add_dep  kinect  user_image_to_rgb.h
add_dep  kinect  skeleton_utils.h
add_dep  kinect  kinect_openni_utils.h
add_dep  kinect  std_utils.h
add_dep  kinect  color_utils_ros.h

#~ add_dep vision voronoi.h
#~ add_dep vision image_contour.h
#~ add_dep vision nite_subscriber_template.h
#~ add_dep vision drawing_utils.h
#~ add_dep vision resize_utils.h
#~ add_dep vision titlemaps.h
#~ add_dep vision content_processing.h
#~ add_dep vision io.h
#~ add_dep kinect user_image_to_rgb.h
#~ add_dep kinect skeleton_utils.h
#~ add_dep kinect kinect_openni_utils.h
#~ add_dep kinect nite_utils.h
#~ add_dep utils color_utils_ros.h
#~ add_dep utils pt_utils.h
#~ add_dep utils geometry_utils.h
#~ add_dep utils hausdorff_distances.h
#~ add_dep utils rect_utils.h
#~ add_dep utils distances.h
#~ add_dep utils combinatorics_utils.h
#~ add_dep utils color_utils.h
#~ add_dep utils foo_point.h
#~ add_dep utils cmatrix.h
#~ add_dep utils string_utils_ros.h
#~ add_dep utils.cpp
#~ add_dep utils.h
#~ add_dep utils extract_utils.h
#~ add_dep utils system_utils.h
#~ add_dep utils timer.h
#~ add_dep utils error.h
#~ add_dep utils debug.h
#~ add_dep compressed_rounded_image_transport  cv_conversion_float_uchar.h
#~ add_dep compressed_rounded_image_transport  hue_utils.h
#~ add_dep compressed_rounded_image_transport  timer_ros.h
#~ add_dep compressed_rounded_image_transport  nan_handling.h
#~ add_dep compressed_rounded_image_transport  min_max.h
remove_include utils_options.h
echo -e "#define LONG_TERM_MEMORY_DIR \"\"\n#define IMG_DIR \"\"\n" > $SRCDIR/img_path.h ; shorten_include img_path.h

echo ; echo "Adding messages..."
mkdir --parents $PKGDIR/msg
roscp kinect NiteSkeletonJoint.msg $PKGDIR/msg
roscp kinect NiteSkeleton.msg $PKGDIR/msg
roscp kinect NiteSkeletonList.msg $PKGDIR/msg
sed -i "s,kinect/NiteSkeleton,$PROJECT/NiteSkeleton," $PKGDIR/msg/*.*
sed -i "s,kinect::NiteSkeleton,$PROJECT::NiteSkeleton," $PKGDIR/src/*.*
shorten_include NiteSkeletonList.h
shorten_include NiteSkeletonJointt.h
shorten_include NiteSkeleton.h

echo ; echo "Adding launch files..."
mkdir --parents $PKGDIR/launch
roscp people_recognition_vision height_detector.launch $PKGDIR/launch
roscp kinect nite_node.launch $PKGDIR/launch
sed -i "s,people_msgs,$PROJECT," $PKGDIR/launch/*.*
sed -i "s,\$(find kinect),\$(find $PROJECT)," $PKGDIR/launch/*.*
sed -i "s,\"kinect\",\"$PROJECT\"," $PKGDIR/launch/*.*
#~ cat $PKGDIR/launch/*.* ; exit

### CMakeLists
echo ; echo "Creating CMakeLists, AUTHORS, INSTALL..."
echo "cmake_minimum_required(VERSION 2.4.6)
include(\$ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)

rosbuild_init()

#set the default path for built executables to the bin directory
set(EXECUTABLE_OUTPUT_PATH \${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the lib directory
set(LIBRARY_OUTPUT_PATH \${PROJECT_SOURCE_DIR}/lib)

### build the custom messages and services
rosbuild_genmsg()
INCLUDE_DIRECTORIES(\${PROJECT_SOURCE_DIR}/msg_gen/cpp/include/$PROJECT)

### deps
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(/usr/include/ni)
INCLUDE_DIRECTORIES(/usr/include/nite)
INCLUDE_DIRECTORIES(/usr/include/openni)

ROSBUILD_ADD_EXECUTABLE( height_pplm src/height_pplm.cpp src/height_pplm.h)
TARGET_LINK_LIBRARIES  ( height_pplm \${OpenCV_LIBS} OpenNI XnVNite)
ROSBUILD_ADD_EXECUTABLE( nite_primitive_standalone src/nite_primitive_standalone.cpp src/nite_primitive.h)
TARGET_LINK_LIBRARIES  ( nite_primitive_standalone OpenNI XnVNite)
ROSBUILD_ADD_EXECUTABLE( publish_kinect_serial src/publish_kinect_serial.cpp)
TARGET_LINK_LIBRARIES  ( publish_kinect_serial)
" > $PKGDIR/CMakeLists.txt

### AUTHORS
echo "
author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
date        `date +%F`
" > $PKGDIR/AUTHORS

### INSTALL
echo "
DOC

It requires OpenCV and curl.

** Compiling

You need to install OpenCV first.

Then, the project is based on a CMakeLists.
For Linux/Mac users, the classical sequence should work:
mkdir build
cd build
cmake ..
make
For Windows users, some instructions are available on OpenCV website:
http://opencv.willowgarage.com/wiki/Getting_started .

** Synopsis and example

Run the generated executable 'height_pplm' with no arguments.
" > $PKGDIR/INSTALL

### list all files
#~ ls $PKGDIR -lh
tree -hD $PKGDIR
du -sh $PKGDIR
#~ exit

### compile
echo ; echo "Trying to compile..."
rosmake $PROJECT
if [ $? -ne 0 ]; then # error while building package
  echo "Error while building $PROJECT! Exiting."
  exit -1;
fi # error after make test

### create tar
TARFILE=$PKGDIR/../$PROJECT.tar.gz
echo ; echo "Generating tar file $TARFILE..."
cd $PKGDIR/..
tar --exclude='build' --exclude='bin' --exclude='lib' --exclude='msg_gen'  --exclude="src/$PROJECT" \
    --create --gzip --verbose --file $TARFILE $PROJECT

# clean
rm $PKGDIR -rf
rospack profile >> /dev/null

echo ; echo "Generated file $TARFILE, finished."
