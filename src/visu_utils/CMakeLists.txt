add_executable(test_mini_stage.exe test_mini_stage.cpp
                                 mini_stage.h mini_stage_plugins.h)
target_link_libraries(test_mini_stage.exe string_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
