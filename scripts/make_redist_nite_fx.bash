#!/bin/bash
# this script will make a standalone package for files
# nite_fx.* and color_face_detector.*
# As such, they can be compiled without other AD dependencies.

# load ROS stuff for being able to use "roscp"
PREV_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH         # store value of ROS_PACKAGE_PATH
source /opt/ros/`rosversion -d`/setup.bash      # load roscp, roscd, etc.
export ROS_PACKAGE_PATH=$PREV_ROS_PACKAGE_PATH # restore value of ROS_PACKAGE_PATH

# output folder
PROJECT=nite_fx
OUTDIR=/tmp/$PROJECT

################################################################################
################################################################################

shorten_include() { # 1:file
  #~ sed -i "s,#include.*[/<]$1.,#include \"$1\",g" $OUTDIR/*.*
  find $OUTDIR  \( -name "*.cpp" -o -name "*.h" \) -exec    \
      sed -i "s,#include.*[/<]$1.,#include \"$1\",g" {} \;
}

add_dep() { # 1:package, 2:file
  ret=`roscp $1 $2 $OUTDIR`
  if [[ $ret != "" ]]
  then
    echo "Failed to copy file '$2' from package '$1': $ret"
    exit
  fi
  shorten_include $2
}

remove_include() { # 1:file
  #~ sed -i "s,#include .*[\"</].*$1[\">],,g" $OUTDIR/*.*
  find $OUTDIR  \( -name "*.cpp" -o -name "*.h" \) -exec    \
      sed -i "s,#include .*[\"</].*$1[\">],,g" {} \;
}


################################################################################
################################################################################

echo ; echo "Creating folder $OUTDIR..."
mkdir --parents $OUTDIR
rm $OUTDIR/* -rf

echo ; echo "Adding classes..."
add_dep kinect        nite_fx.cpp
add_dep kinect        nite_primitive.h

# effect collection
roscd vision/vision/skills/
cp dancer $OUTDIR -r
rm $OUTDIR/dancer/.svn $OUTDIR/dancer/flying_game -rf
shorten_include  effect_collection.h

# deps
echo ; echo "Adding deps and changing include paths..."
# from most specifici to most generic: a file should only depend on the following ones
add_dep vision_utils                         layer_utils.h   # for effect_collection_nite_fx
add_dep vision_utils                         value_remover.h   # for effect_collection_nite_fx
add_dep vision_utils                         border_remover.h   # for effect_collection_nite_fx
add_dep vision_utils                         drawing_utils.h   # for effect_collection_nite_fx
add_dep vision_utils                         resize_utils.h   # for effect_collection_nite_fx
add_dep vision_utils                         titlemaps.h   # for effect_collection_nite_fx
add_dep vision_utils                         disjoint_sets2.cpp   # for effect_collection_nite_fx
add_dep vision_utils                         disjoint_sets2.h   # for effect_collection_nite_fx
add_dep vision_utils                         comp_labeller_interface.h   # for effect_collection_nite_fx
add_dep kinect                               skeleton_utils.h
add_dep kinect                               NiteSkeletonLite.h
add_dep kinect                               user_image_to_rgb.h
add_dep utils                                geometry_utils.h # for effect_collection_nite_fx
add_dep utils                                distances.h # for effect_collection_nite_fx
add_dep utils                                hausdorff_distances.h # for effect_collection_nite_fx
add_dep utils                                rect_utils.h # for effect_collection_nite_fx
add_dep utils                                combinatorics_utils.h # for effect_collection_nite_fx
add_dep utils                                color_utils.h
add_dep utils                                foo_point.h
add_dep utils                                StringUtils.h # for effect_collection_nite_fx
add_dep utils                                timer.h
add_dep utils                                error.h # for effect_collection_nite_fx
add_dep utils                                debug.h
add_dep compressed_rounded_image_transport   hue_utils.h
add_dep compressed_rounded_image_transport   cv_conversion_float_uchar.h # for effect_collection_nite_fx
add_dep compressed_rounded_image_transport   nan_handling.h # for effect_collection_nite_fx
add_dep compressed_rounded_image_transport   min_max.h # for effect_collection_nite_fx

#~ remove_include                               StringUtils.h
remove_include                               utils_options.h
#~ remove_include                               img_path.h
echo -e "#define LONG_TERM_MEMORY_DIR \"$OUTDIR/\"\n" > $OUTDIR/img_path.h ; shorten_include img_path.h
#~ remove_include                               error.h

# config files
roscp kinect openni_tracker.xml $OUTDIR

# video_backgrounds
VIDEODIR=$OUTDIR/video_backgrounds
mkdir --parents $VIDEODIR
roscp vision_utils news.m4v $VIDEODIR

### CMakeLists
echo ; echo "Creating CMakeLists, AUTHORS, INSTALL..."
echo "
PROJECT( nite_fx_proj )
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)

FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(\"/usr/include/ni\")
INCLUDE_DIRECTORIES(\"/usr/include/nite\")
INCLUDE_DIRECTORIES(\"/usr/include/openni\")
INCLUDE_DIRECTORIES(\${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(dancer)

ADD_LIBRARY(disjoint_sets2 disjoint_sets2.cpp disjoint_sets2.h)
ADD_LIBRARY(effect_collection_nite_fx dancer/effect_collection_nite_fx.cpp)
TARGET_LINK_LIBRARIES( effect_collection_nite_fx \${OpenCV_LIBS} disjoint_sets2)

ADD_EXECUTABLE( nite_fx nite_fx.cpp nite_primitive.h)
TARGET_LINK_LIBRARIES( nite_fx OpenNI XnVNite effect_collection_nite_fx)

" > $OUTDIR/CMakeLists.txt

### AUTHORS
echo "
author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
date        `date +%F`
" > $OUTDIR/AUTHORS

### INSTALL
echo "

Generate visual FX for the Kinect.
It makes use of the data supplied by the Kinect: RGB, depth, user map, and skeleton calibration.

** Dependencies:

It requires OpenCV, OpenNI, NiTE.

You need to install OpenCV first.
It is available in repositories for Linux.
For Windows, go to
http://opencv.org/downloads.html

Then install OpenNI and NITE.
For Linux, detailed instructions are available in
https://sites.google.com/site/rameyarnaud/misc/c/openni-and-nite-1-5-installation
For Windows, follow
www.openni.org/openni-sdk/openni-sdk-history-2/

** Compiling

The project is based on a CMakeLists.
For Linux/Mac users, the classical sequence should work:
______
mkdir build
cd build
cmake ..
make
______
For Windows users, some instructions are available on OpenCV website:
http://opencv.willowgarage.com/wiki/Getting_started .

** Synopsis and example

From the main fodler, run the generated executable 'build/nite_fx' with no arguments.
" > $OUTDIR/INSTALL

### list all files
tree -hD $OUTDIR
du -sh $OUTDIR

### compile
echo ; echo "Trying to compile..."
mkdir --parents $OUTDIR/build
cd $OUTDIR/build
cmake ..
#~ make -j 3
make
if [ $? -ne 0 ]; then # error while building package
  echo "Error while building $PROJECT! Exiting."
  exit -1;
fi # error after make test

### create tar
TARFILE=/tmp/$PROJECT.tar.gz
echo ; echo "Generating tar file $TARFILE..."
tar --exclude='build' --directory /tmp --create --gzip --verbose --file $TARFILE $PROJECT

echo ; echo "Generated file $TARFILE, finished."
