# The script was tested with autoconf 2.65
AC_PREREQ([2.63])

# Define the package name, version, bugreport mail, tarname, URL
AC_INIT([easykf],[2.03],[jeremy.fix@gmail.com],[easykf],[http://jeremy.fix.free.fr])

# Just a sanity check to ensure the configure.ac corresponds to the current project
AC_CONFIG_SRCDIR([src/ukf.h])
AC_CONFIG_MACRO_DIR([m4])
# Options (such as different formats for archive can be specified to AM_INIT_AUTOMAKE , see chap 17 http://www.gnu.org/software/automake/manual/automake.html
AM_INIT_AUTOMAKE

# Check for a C++ compiler
AC_PROG_CXX
#AC_SEARCH_LIBS([strerror],[cposix])
#AC_USE_SYSTEM_EXTENSIONS

# Overload CXXFLAGS
CXXFLAGS="-Wall -Wfloat-equal -O2 -ansi -pedantic"

# Check for some standard header files (stdlib.h, stdarg.h, string.h,..)
AC_HEADER_STDC
# Check for the module gsl
# and set the cflags and libs in : sl_variables_CFLAGS and sl_variables_LIBS
# used in the Makefile.am 
PKG_CHECK_MODULES([easykf_variables], [gsl] )

LT_INIT

# Check if doxygen is installed
# if not, set DOXYGEN=no

AC_PATH_PROG(DOXYGEN, doxygen, no)

# Set in DOXY if doxygen was found
# used, in doc/Makefile.am to trigger or not the creation of the doc
AM_CONDITIONAL([DOXY], [test "$DOXYGEN" != "no"])

DX_DOXYGEN_FEATURE(ON)
DX_MAN_FEATURE(OFF)
DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)
# This line will be used in ./doc/Makefile.am
# so the path provided to DX_INIT_DOXYGEN is relative to doc
DX_INIT_DOXYGEN(easykf, Doxyfile, .)


AC_CONFIG_FILES([
easykf.spec
easykf.pc
Makefile
doc/Doxyfile
doc/Makefile
doc/eps/Makefile
m4/Makefile
src/Makefile
examples/Makefile
examples/Data/Makefile
examples/Output/Makefile
])
AC_OUTPUT

# We add after AC_OUTPUT, processings done after executing config.status
# such as warning messages complaining the absence of doxygen
if test "$DOXYGEN" == "no"; then
   AC_MSG_WARN([Cannot find doxygen, the documentation will not be built; Check www.doxygen.org for installing Doxygen])
fi
