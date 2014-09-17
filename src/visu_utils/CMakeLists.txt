
IF(AD_USE_OPENCV)
  add_executable(test_GraphMaker.exe test_GraphMaker.cpp)
  target_link_libraries(test_GraphMaker.exe ${catkin_LIBRARIES})

  add_executable(test_mini_stage.exe test_mini_stage.cpp 
                                     mini_stage.h mini_stage_plugins.h)
  target_link_libraries(test_mini_stage.exe string_utils ${catkin_LIBRARIES})
 
  add_executable(test_histogram_utils.exe test_histogram_utils.cpp
                                          histogram_utils.h)
   
  add_executable(test_array_to_color.exe test_array_to_color.cpp 
                                         array_to_color.h)
  
  add_executable(test_pie_chart_utils.exe test_pie_chart_utils.cpp 
                                          pie_chart_utils.h)
  
  add_executable(test_timer_chart.exe test_timer_chart.cpp 
                                      timer_chart.h)
ENDIF(AD_USE_OPENCV)
