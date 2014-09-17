#!/bin/bash
# this script will make a standalone package for files
# artoolkit_utils.* and color_face_detector.*
# As such, they can be compiled without other AD dependencies.

# load ROS stuff for being able to use "roscp"
PREV_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH         # store value of ROS_PACKAGE_PATH
source /opt/ros/`rosversion -d`/setup.bash      # load roscp, roscd, etc.
export ROS_PACKAGE_PATH=$PREV_ROS_PACKAGE_PATH # restore value of ROS_PACKAGE_PATH

# output folder
PROJECT=artoolkit_utils
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
add_dep people_detection_vision artoolkit_img2patt.cpp
add_dep people_detection_vision artoolkit_patt2img.cpp
#~ add_dep utils         StringUtils.cpp
#~ add_dep utils         StringUtils.h
add_dep utils         filename_handling.h
add_dep utils         string_split.h
add_dep utils         find_and_replace.h
add_dep utils         string_casts.h
add_dep utils         file_io.h

# deps
echo ; echo "Adding deps and changing include paths..."
# from most specifici to most generic: a file should only depend on the following ones
add_dep vision_utils  border_remover.h
add_dep utils         extract_utils.h
add_dep utils         system_utils.h
add_dep utils         error.h
add_dep utils         debug.h
remove_include        utils_options.h

# sample images
add_dep people_detection_vision alice.png
add_dep people_detection_vision carrefour_card.patt

### CMakeLists
echo ; echo "Creating CMakeLists, AUTHORS, INSTALL..."
echo "
PROJECT( artoolkit_utils_proj )
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)

ADD_EXECUTABLE( artoolkit_img2patt artoolkit_img2patt.cpp)
ADD_EXECUTABLE( artoolkit_patt2img artoolkit_patt2img.cpp)
TARGET_LINK_LIBRARIES( artoolkit_img2patt \${OpenCV_LIBS} \${Boost_LIBS})
TARGET_LINK_LIBRARIES( artoolkit_patt2img \${OpenCV_LIBS})

" > $OUTDIR/CMakeLists.txt

### AUTHORS
echo "
author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
date        `date +%F`
" > $OUTDIR/AUTHORS

### INSTALL
echo "

ToolKit ( http://www.hitl.washington.edu/artoolkit/ ) is a computer tracking
library for creation of augmented reality applications that overlay virtual
imagery on the real world.

It works with so-called pattern files, representing the visual appearance of
the markers. These patterns are then searched in the input stream of image.

Two programs are given here:
* One can transform color or BW images into ARToolkit patterns.
* One can transform ARToolKit patterns into images for visualizing them

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

Run the generated executables 'artoolkit_img2patt'
or 'artoolkit_patt2img' with no arguments.
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
#~ make 
if [ $? -ne 0 ]; then # error while building package
  echo "Error while building $PROJECT! Exiting."
  exit -1;
fi # error after make test

### create tar
TARFILE=/tmp/$PROJECT.tar.gz
echo ; echo "Generating tar file $TARFILE..."
tar --exclude='build' --directory /tmp --create --gzip --verbose --file $TARFILE $PROJECT

echo ; echo "Generated file $TARFILE, finished."
