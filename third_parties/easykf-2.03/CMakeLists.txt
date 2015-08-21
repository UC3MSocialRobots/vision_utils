cmake_minimum_required(VERSION 2.6.1)
PROJECT(easykf CXX)
SET(COMPIlE_EXAMPLES OFF)

SET(EASY_KF_HEADERS
    src/ukf.h
    src/ukf_types.h
    src/ukf_math.h
    src/ukf_samples.h
    src/ukf_parameter_scalar.h
    src/ukf_parameter_ndim.h
    src/ukf_state_ndim.h
    src/ukf_sr_state_ndim.h
    src/ekf_types.h
    src/ekf.h
)
INCLUDE_DIRECTORIES(src)

IF(COMPILE_EXAMPLES)
  ADD_EXECUTABLE(example-001
                 examples/example-001.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-001 gsl blas)

  ADD_EXECUTABLE(example-002
                 examples/example-002.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-002 gsl blas)

  ADD_EXECUTABLE(example-003
                 examples/example-003.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-003 gsl blas)

  ADD_EXECUTABLE(example-004
                 examples/example-004.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-004 gsl blas)

  ADD_EXECUTABLE(example-005
                 examples/example-005.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-005 gsl blas)

  ADD_EXECUTABLE(example-006
                 examples/example-006.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-006 gsl blas)

  ADD_EXECUTABLE(example-007
                 examples/example-007.cc ${EASY_KF_HEADERS})
  target_link_libraries(example-007 gsl blas)

  add_executable(kf_foo.exe examples/foo.cpp ${EASY_KF_HEADERS})
  target_link_libraries(kf_foo.exe gsl blas)
ENDIF(COMPILE_EXAMPLES)
