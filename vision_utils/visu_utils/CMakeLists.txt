
IF(AD_USE_OPENCV)
  add_maggie_test(test_GraphMaker.exe
      SOURCES     test_GraphMaker.cpp
      LINK        opencv
  )

  add_maggie_test(test_mini_stage.exe
      SOURCES     test_mini_stage.cpp mini_stage.h mini_stage_plugins.h
      LINK        string_utils
  )

  add_maggie_test(test_histogram_utils.exe
      SOURCES     test_histogram_utils.cpp histogram_utils.h
      LINK
  )

  add_maggie_test(test_array_to_color.exe
      SOURCES     test_array_to_color.cpp array_to_color.h
      LINK
  )

  add_maggie_test(test_pie_chart_utils.exe
      SOURCES     test_pie_chart_utils.cpp pie_chart_utils.h
      LINK
  )

  add_maggie_test(test_timer_chart.exe
      SOURCES     test_timer_chart.cpp timer_chart.h
      LINK
  )
ENDIF(AD_USE_OPENCV)
