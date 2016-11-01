# `vision_utils`

[![Build Status](https://travis-ci.org/UC3MSocialRobots/vision_utils.svg)](https://travis-ci.org/UC3MSocialRobots/vision_utils)

`vision_utils` is a [ROS](http://www.ros.org/) package
that supplies a variety of tools and utilities to
manipulate color images, depth images, user masks, and more.

How to install
==============

## 1. Dependencies from sources

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Run the following instructions:

```bash
$ sudo apt-get install python-wstool
$ wstool init
$ wstool merge `rospack find vision_utils`/dependencies.rosinstall
$ roscd ; cd src
$ wstool update
```
## 2. Dependencies included in the Ubuntu packages

Please run the [rosdep](http://docs.ros.org/independent/api/rosdep/html/) utility:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep install vision_utils --ignore-src
```

Some dependencies must be installed manually as they are not included
in ```rosdep```:

```bash
$ sudo apt install tesseract-ocr-fra  tesseract-ocr-spa
```
