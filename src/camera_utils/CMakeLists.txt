IF(AD_USE_OPENCV)
  add_executable(opencv_calibration.exe opencv_calibration.cpp)
  target_link_libraries(opencv_calibration.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

  IF(AD_USE_ROS)
	add_executable(image_topic_to_video.exe image_topic_to_video.cpp)
        target_link_libraries(image_topic_to_video.exe string_utils ${catkin_LIBRARIES})

	add_executable(rgb_and_depth_to_yaml.exe rgb_and_depth_to_yaml.cpp)
        target_link_libraries(rgb_and_depth_to_yaml.exe string_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})

	add_executable(ros_camera_info_to_binary.exe image_topic_to_video.cpp)
        target_link_libraries(ros_camera_info_to_binary.exe string_utils ${catkin_LIBRARIES})
  ENDIF(AD_USE_ROS)

	add_executable(launcher_camera_blocker_skill.exe launcher_camera_blocker_skill.cpp)
        target_link_libraries(launcher_camera_blocker_skill.exe string_utils ${catkin_LIBRARIES})
	
	add_library(camera_utils camera_utils.cpp)
        target_link_libraries(camera_utils ${catkin_LIBRARIES})
	
	add_executable(test_camera_utils.exe test_camera_utils.cpp)
	target_link_libraries(test_camera_utils.exe camera_utils)
	
	install(TARGETS camera_utils
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
ENDIF(AD_USE_OPENCV)


