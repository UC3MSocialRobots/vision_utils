IF(AD_USE_NITE)

    add_executable(foo_nite_subscriber.exe foo_nite_subscriber.cpp
                                       nite_subscriber_template.h)
    target_link_libraries(foo_nite_subscriber.exe string_utils OpenNI
                                                  ${catkin_LIBRARIES}) #XnVNite)

ENDIF(AD_USE_NITE)
