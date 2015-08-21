#This generates Makefile.in by the use of automake.
ACLOCAL_AMFLAGS = -I m4

# extra tuff that need to be included in the package.
EXTRA_DIST = \
	reconf \
	configure \
	easykf.spec

# The subdirectories.
SUBDIRS = \
	m4 \
	src \
	doc\
	examples

# This is the path where .pc files will be put.
pkgconfigdir = $(libdir)/pkgconfig

# This is what to put in the previous path.
pkgconfig_DATA= easykf.pc
