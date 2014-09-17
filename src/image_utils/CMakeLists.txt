
### opencv_face_detector
add_executable(test_opencv_face_detector.exe 
			   test_opencv_face_detector.cpp opencv_face_detector.h)
target_link_libraries(test_opencv_face_detector.exe ${catkin_LIBRARIES})


### border_remover
add_executable(test_border_remover.exe test_border_remover.cpp border_remover.h)
target_link_libraries(test_border_remover.exe ${catkin_LIBRARIES})

###
add_executable(test_google_images_search.exe
			   test_google_images_search.cpp google_images_search.h)
target_link_libraries(test_google_images_search.exe string_utils ${catkin_LIBRARIES})

###
add_executable(test_image_utils.exe test_image_utils.cpp
                                    layer_utils.h
                                    content_processing.h connected_comp_interface.h
                                    value_remover.h cv_border_closer.h  titlemaps.h)
target_link_libraries(test_image_utils.exe disjoint_sets2 string_utils ${catkin_LIBRARIES})

add_executable(test_drawing_utils.exe test_drawing_utils.cpp  drawing_utils.h)
target_link_libraries(test_drawing_utils.exe ${catkin_LIBRARIES})

add_executable(test_io.exe test_io.cpp io.h)
target_link_libraries(test_io.exe ${catkin_LIBRARIES})

# MCP tests
add_executable(test_voronoi.exe test_voronoi.cpp voronoi.h)
target_link_libraries(test_voronoi.exe ${catkin_LIBRARIES})
