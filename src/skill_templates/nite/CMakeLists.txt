IF(AD_USE_NITE)
    add_library(foo_nite_nodelet_subscriber foo_nite_nodelet_subscriber.cpp
                                      nite_nodelet_subscriber_template.h)

    add_executable(foo_nite_subscriber.exe foo_nite_subscriber.cpp
                                       nite_subscriber_template.h)
    target_link_libraries(foo_nite_subscriber.exe string_utils OpenNI
                                                  XnVNite ${catkin_LIBRARIES})

    install(TARGETS foo_nite_nodelet_subscriber
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
ENDIF(AD_USE_NITE)
