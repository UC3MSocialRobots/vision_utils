### opencv_face_detector
###
add_executable(test_drawing_utils.exe test_drawing_utils.cpp  )
target_link_libraries(test_drawing_utils.exe ${catkin_LIBRARIES})
# MCP tests
add_executable(test_voronoi.exe test_voronoi.cpp )
target_link_libraries(test_voronoi.exe ${catkin_LIBRARIES})
