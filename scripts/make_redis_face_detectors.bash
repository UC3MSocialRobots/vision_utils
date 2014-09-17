#!/bin/bash
# this script will make a standalone package for files
# texture_sensor.* and color_face_detector.*
# As such, they can be compiled without other AD dependencies.

# load ROS stuff for being able to use "roscp"
PREV_ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH         # store value of ROS_PACKAGE_PATH
source /opt/ros/`rosversion -d`/setup.bash      # load roscp, roscd, etc.
export ROS_PACKAGE_PATH=$PREV_ROS_PACKAGE_PATH # restore value of ROS_PACKAGE_PATH

# output folder
PROJECT=face_detectors
OUTDIR=/tmp/$PROJECT

# files where to copy functions with copy_function()
INCFILE=$OUTDIR/includes.h

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
    echo "Failed to copy file $2 from package $1: $ret"
    exit
  fi
  shorten_include $2
}
remove_include() { # 1:file
  #~ sed -i "s,#include .*[\"</].*$1[\">],,g" $OUTDIR/*.*
  find $OUTDIR  \( -name "*.cpp" -o -name "*.h" \) -exec    \
    sed -i "s,#include .*[\"</].*$1[\">],,g" {} \;
}

remove_include_package() { # 1:file
  sed -i "s,#include [\"<]$1[/\.].*.,,g" $OUTDIR/*.*
}

copy_function() { #1: package  2: file, 3: function
  # grep --line-number -e "^.*create_face_classifier$" ~/manager_ros/vision/vision/utils/image_utils/opencv_face_detector.h | grep --only-matching [0-9]*
  # grep --line-number -e "^.*end create_face_classifier.*$" ~/manager_ros/vision/vision/utils/image_utils/opencv_face_detector.h | grep --only-matching [0-9]*
  # grep --line-number -e "create_face_classifier.*" ~/manager_ros/vision/vision/utils/image_utils/opencv_face_detector.h | grep --only-matching [0-9]*
  # sed -n 35,41p opencv_face_detector.h
  FILE=/tmp/copy_function.txt
  roscp $1 $2 $FILE
  # match <pattern>(
  BEGIN=`grep --max-count=1 --line-number -e "$3(" $FILE | awk -F ":" '{print $1}'`
  if [[ -z $BEGIN ]]; then
    # match <pattern>\n
    BEGIN=`grep --max-count=1 --line-number -e "^.*$3$" $FILE | awk -F ":" '{print $1}'`
  fi
  if [[ -z $BEGIN ]]; then
    echo "Could not find the beginning of function '$3' in $2 (copied in $FILE)"
    exit
  fi

  # iterate backward as long as line not empty
  #while true
  while true; do
    LINE=`sed -n ${BEGIN},${BEGIN}p $FILE`
    #echo "$BEGIN:$LINE"
    if [[ $LINE == "" ]]; then
      break
    fi
    BEGIN=$(($BEGIN - 1))
  done

  END=`grep --max-count=1 --line-number -e "end $3()" $FILE | awk -F ":" '{print $1}'`
  if [[ -z $END ]]; then
    echo "Could not find the end of function '$3' in $2 (copied in $FILE)"
    exit
  fi

  echo ; echo "** Copying function $3 (lines $BEGIN -> $END of $2)"

  sed -n ${BEGIN},${END}p $FILE >> $INCFILE
  echo "//////////////////////////" >> $INCFILE
}

insert_first_include() { # 1: file to modify, 2: filename to insert
  LINE=`grep --max-count=1 --line-number -e "#include" $1 | awk -F ":" '{print $1}'`
  if [[ -z $LINE ]]; then
    echo "Could not find #include in $1"
    exit
  fi
  echo LINE:$LINE
  # insert text at given line
  # cf http://stackoverflow.com/questions/3273670/inserting-text-into-a-specific-line
  #awk 'NR==5{$0="!comment: http://www.test.com\n"$0}1' $1
  INSTR="'NR==5{\$0=\"!comment: http://www.test.com\n\"\$0}1'"
  echo INSTR $INSTR
  awk '$INSTR' $1
}

################################################################################
################################################################################

### tests for copy_function
#~ copy_function vision opencv_face_detector.h create_face_classifier
#~ echo "Expected 33 -> 41"
#~ copy_function vision opencv_face_detector.h detect_with_opencv
#~ echo "Expected 44 -> 89"
#~ copy_function vision array_to_color.h array_to_color
#~ echo "Expected 58 -> 161"
#~ copy_function vision array_to_color.h array_to_col2
#~ exit

#~ insert_first_include /tmp/redis/texture_sensor.h foo.h
#~ exit

echo "Creating folder $OUTDIR..."
mkdir --parents $OUTDIR
rm $OUTDIR/* -rf
touch $INCFILE

echo "Adding classes..."
add_dep people_msgs texture_sensor.cpp
add_dep people_msgs texture_sensor.h
add_dep people_msgs color_face_detector.cpp
add_dep people_msgs color_face_detector.h
add_dep vision test_opencv_face_detector.cpp
add_dep vision opencv_face_detector.h

# deps
echo "Adding deps and changing include paths..."
# from most specifici to most generic: a file should only depend on the following ones
add_dep vision array_to_color.h
add_dep vision drawing_utils.h
add_dep vision resize_utils.h
add_dep vision titlemaps.h
#~ add_dep vision io.h
add_dep utils stats_utils.h
add_dep utils geometry_utils.h
add_dep utils hausdorff_distances.h
add_dep utils rect_utils.h
add_dep utils distances.h
add_dep utils combinatorics_utils.h
add_dep utils colormaps.h
add_dep utils color_utils.h
add_dep utils foo_point.h
add_dep utils system_utils.h
add_dep utils StringUtils.h
add_dep utils gnuplot_i.hpp
add_dep utils clamp.h
add_dep utils timer.h
add_dep utils error.h
add_dep utils debug.h
add_dep compressed_rounded_image_transport hue_utils.h
add_dep compressed_rounded_image_transport cv_conversion_float_uchar.h
add_dep compressed_rounded_image_transport min_max.h
add_dep compressed_rounded_image_transport nan_handling.h
add_dep compressed_rounded_image_transport timer_ros.h
touch $OUTDIR/utils_options.h ; shorten_include utils_options.h
echo "#define IMG_DIR " > $OUTDIR/img_path.h ; shorten_include img_path.h

#~ add_dep utils gnuplot_i.hpp
#~
#~ copy_function utils stats_utils.h mean
#~
#~ remove_include_package vision
#~ remove_include_package utils
#~ remove_include_package compressed_rounded_image_transport
#~ remove_include_package cv_conversion_float_uchar
#~ add_dep utils gnuplot_i.hpp
#~ echo "#define IMG_DIR " > $OUTDIR/img_path.h ; shorten_include img_path.h

### CMakeLists
echo "Creating CMakeLists, AUTHORS, INSTALL..."
echo "
PROJECT( texture_sensor_proj )
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
FIND_PACKAGE( OpenCV REQUIRED )

ADD_EXECUTABLE( texture_sensor texture_sensor.cpp texture_sensor.h)
TARGET_LINK_LIBRARIES( texture_sensor \${OpenCV_LIBS} )

ADD_EXECUTABLE( color_face_detector color_face_detector.cpp  color_face_detector.h)
TARGET_LINK_LIBRARIES( color_face_detector \${OpenCV_LIBS} )

ADD_EXECUTABLE( test_opencv_face_detector test_opencv_face_detector.cpp  opencv_face_detector.h)
TARGET_LINK_LIBRARIES( test_opencv_face_detector \${OpenCV_LIBS} )
" > $OUTDIR/CMakeLists.txt

### AUTHORS
echo "
author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
date        `date +%F`
" > $OUTDIR/AUTHORS

### INSTALL
echo "
A watermark remover.
It needs a clean thumbnail of the image.
It will replace the watermarked zones of the image
with the ones of the thumbnail.

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

** Synopsis

...
** Example

...
" > $OUTDIR/INSTALL

### list all files
tree -hD $OUTDIR
du -sh $OUTDIR

### compile
echo "Trying to compile..."
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
