# Copyright (C) 1999-2005 Open Source Telecom Corporation.
#
# This file is free software; as a special exception the author gives
# unlimited permission to copy and/or distribute it, with or without
# modifications, as long as this notice is preserved.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY, to the extent permitted by law; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

include ../aminclude.am

SUBDIRS = eps

MOSTLYCLEANFILES = $(DX_CLEANFILES)
if DOXY
all: doxygen-doc
install: doxygen-doc install-docDATA
endif

MAINTAINERCLEANFILES = Makefile.in Makefile
EXTRA_DIST = $(DX_CONFIG) $(DX_CONFIG).in DoxygenLayout.xml

#
# Directory where to install the documentation
#
docdir = $(DESTDIR)/$(datadir)/doc/$(PACKAGE_NAME)-$(PACKAGE_VERSION)
doc_DATA = $(srcdir)/html
install-docDATA: 
	mkdir -p $(docdir)
	cp -r $(doc_DATA) $(docdir)/

#
# Likewise uninstall
#
uninstall-docDATA:
	rm -rf $(docdir)
#
# Clean everything (uninstall and then local clean)
#
distclean: clean uninstall-docDATA 


