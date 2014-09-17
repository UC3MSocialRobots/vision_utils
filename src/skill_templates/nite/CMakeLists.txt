IF(AD_USE_NITE)

  rosbuild_add_library(foo_nite_nodelet_subscriber foo_nite_nodelet_subscriber.cpp
                                                    nite_nodelet_subscriber_template.h)

  rosbuild_add_executable(foo_nite_subscriber   foo_nite_subscriber.cpp
                                                nite_subscriber_template.h)
  target_link_libraries(foo_nite_subscriber     string_utils OpenNI XnVNite)
ENDIF(AD_USE_NITE)
