#!/bin/sh
# comment all the "#define DISPLAY" lines in
sed -i 's,^#define DISPLAY$,//#define DISPLAY,g' *.cpp
