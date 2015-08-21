clean-local:
	rm -rf Output/*.png Output/*.ps

SUBDIRS = Data Output

#install-binPROGRAMS: # Disable the installation of the examples
#uninstall-binPROGRAMS:

bin_PROGRAMS = \
	easykf-example-001-xor\
	easykf-example-002-xor\
	easykf-example-003-RBF\
	easykf-example-004-MLP\
	easykf-example-005-Mackay\
	easykf-example-006-Rosenbrock\
	easykf-example-007-Lorentz\
	easykf-example-008-SR-Lorentz\
	easykf-example-009-EKF-Lorentz

easykf_example_001_xor_SOURCES  = \
	example-001.cc
easykf_example_001_xor_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_001_xor_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_002_xor_SOURCES  = \
	example-002.cc
easykf_example_002_xor_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_002_xor_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_003_RBF_SOURCES  = \
	example-003.cc
easykf_example_003_RBF_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_003_RBF_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_004_MLP_SOURCES  = \
	example-004.cc
easykf_example_004_MLP_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_004_MLP_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_005_Mackay_SOURCES  = \
	example-005.cc
easykf_example_005_Mackay_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_005_Mackay_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_006_Rosenbrock_SOURCES  = \
	example-006.cc
easykf_example_006_Rosenbrock_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_006_Rosenbrock_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_007_Lorentz_SOURCES  = \
	example-007.cc
easykf_example_007_Lorentz_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_007_Lorentz_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_008_SR_Lorentz_SOURCES  = \
	example-008.cc
easykf_example_008_SR_Lorentz_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_008_SR_Lorentz_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf

easykf_example_009_EKF_Lorentz_SOURCES  = \
	example-009.cc
easykf_example_009_EKF_Lorentz_CPPFLAGS = -I../src -O2 $(easykf_variables_CFLAGS)
easykf_example_009_EKF_Lorentz_LDFLAGS  = $(easykf_variables_LIBS) -L../src -leasykf
