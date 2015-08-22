add_executable(opencv_calibration.exe opencv_calibration.cpp)
target_link_libraries(opencv_calibration.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(image_topic_to_video.exe image_topic_to_video.cpp)
target_link_libraries(image_topic_to_video.exe  ${catkin_LIBRARIES})

add_executable(launcher_camera_blocker_skill.exe launcher_camera_blocker_skill.cpp)
target_link_libraries(launcher_camera_blocker_skill.exe  ${catkin_LIBRARIES})
