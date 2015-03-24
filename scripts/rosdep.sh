#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="ros-`rosversion -d`-blob
          tesseract-ocr
          tesseract-ocr-eng
          tesseract-ocr-spa
          tesseract-ocr-fra"

sudo apt-get install $PACKAGES
