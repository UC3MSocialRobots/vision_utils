add_executable(test_cv_conversion_float_uchar.exe
              test_cv_conversion_float_uchar.cpp
              float_image_generator.h)
target_link_libraries(test_cv_conversion_float_uchar.exe
                      opencv_features2d ${catkin_LIBRARIES})

add_executable(depth_to_rgb_viewer.exe
               depth_to_rgb_viewer.cpp)
target_link_libraries(depth_to_rgb_viewer.exe ${catkin_LIBRARIES})
