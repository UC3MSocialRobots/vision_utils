#!/bin/bash
# this script will make a standalone package for files
# voronoi.* and color_face_detector.*
# As such, they can be compiled without other AD dependencies.

# load ROS stuff for being able to use "roscp"
PREV_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH         # store value of ROS_PACKAGE_PATH
source /opt/ros/`rosversion -d`/setup.bash      # load roscp, roscd, etc.
export ROS_PACKAGE_PATH=$PREV_ROS_PACKAGE_PATH # restore value of ROS_PACKAGE_PATH

# output folder
PROJECT=voronoi
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
add_dep vision_utils test_voronoi.cpp
add_dep vision_utils voronoi.h

# deps
echo ; echo "Adding deps and changing include paths..."
# from most specifici to most generic: a file should only depend on the following ones
add_dep vision_utils image_contour.h
add_dep utils timer.h
add_dep utils debug.h
remove_include utils_options.h
remove_include img_path.h
remove_include error.h

# sample images
add_dep vision_utils opencv_src.png
add_dep vision_utils alberto1_user_mask.png
add_dep vision_utils horse.png

### CMakeLists
echo ; echo "Creating CMakeLists, AUTHORS, INSTALL..."
echo "
PROJECT( voronoi_proj )
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
FIND_PACKAGE( OpenCV REQUIRED )

ADD_EXECUTABLE( voronoi test_voronoi.cpp voronoi.h)
TARGET_LINK_LIBRARIES( voronoi \${OpenCV_LIBS} )

" > $OUTDIR/CMakeLists.txt

### AUTHORS
echo "
author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
date        `date +%F`
" > $OUTDIR/AUTHORS

### INSTALL
echo "
The morphological skeleton of an image is the set of its non-zero pixels which are equidistant to its boundaries.
More info: http://en.wikipedia.org/wiki/Topological_skeleton

Thinning an image consits in reducing its non-zero pixels to their morphological skeleton.
More info: http://en.wikipedia.org/wiki/Thinning_(morphology)

VoronoiSkeleton is a C++ class
made for the fast computing of Voronoi diagrams of monochrome images.
It contains different implementations of thinning algorithms:

* Zhang - Suen explained in 'A fast parallel algorithm for thinning digital
patterns' by T.Y. Zhang and C.Y. Suen and based on implentation by
http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/

* Guo - Hall explained in 'Parallel thinning with two sub-iteration
algorithms' by Zicheng Guo and Richard Hall and based on implentation by
http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/

* a morphological one, based on the erode() and dilate() operators.
Coming from: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

A special care has been given to optimize the 2 first ones. Instead of
re-examining the whole image at each iteration, only the pixels of the
current contour are considered. This leads to a speedup by almost 100 times
on experimental tests.

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

Run the generated executable 'test_voronoi' with no arguments.
" > $OUTDIR/INSTALL

### list all files
tree -hD $OUTDIR
du -sh $OUTDIR

### compile
echo ; echo "Trying to compile..."
mkdir --parents $OUTDIR/build
cd $OUTDIR/build
cmake ..
make -j 3
if [ $? -ne 0 ]; then # error while building package
  echo "Error while building $PROJECT! Exiting."
  exit -1;
fi # error after make test

### create tar
TARFILE=/tmp/$PROJECT.tar.gz
echo ; echo "Generating tar file $TARFILE..."
tar --exclude='build' --directory /tmp --create --gzip --verbose --file $TARFILE $PROJECT

echo ; echo "Generated file $TARFILE, finished."
