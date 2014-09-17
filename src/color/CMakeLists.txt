### color_utils

add_executable(test_color_utils.exe test_color_utils.cpp
                                    color_utils.h colormaps.h)
target_link_libraries(test_color_utils.exe ${catkin_LIBRARIES})
