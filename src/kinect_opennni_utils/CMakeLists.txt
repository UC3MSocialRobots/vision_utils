add_executable(test_kinect_openni_utils.exe test_kinect_openni_utils.cpp)
target_link_libraries(test_kinect_openni_utils.exe ${PCL_LIBRARIES}
  ${catkin_LIBRARIES} )
