catkin_add_gtest     (gtest_array_to_color  gtest_array_to_color.cpp)
TARGET_LINK_LIBRARIES(gtest_array_to_color  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_blob_segmenter
  gtest_blob_segmenter.cpp
  ../src/point_clouds/blob_segmenter.h
  ../src/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_blob_segmenter string_utils disjoint_sets2 ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_border_remover
  gtest_border_remover.cpp
  ../src/image_utils/border_remover.h)
target_link_libraries(gtest_border_remover ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_tilter
  gtest_cloud_tilter.cpp
  ../src/point_clouds/cloud_tilter.h)
TARGET_LINK_LIBRARIES(gtest_cloud_tilter string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_viewer_gnuplot
  gtest_cloud_viewer_gnuplot.cpp
  ../src/point_clouds/cloud_viewer_gnuplot.h)
TARGET_LINK_LIBRARIES(gtest_cloud_viewer_gnuplot string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_viewer
  gtest_cloud_viewer.cpp
  ../src/point_clouds/cloud_viewer.h)
target_link_libraries(gtest_cloud_viewer ${PCL_LIBRARIES} )

catkin_add_gtest     (gtest_clusterer
  gtest_clusterer.cpp
  ../src/point_clouds/clusterer.h)
target_link_libraries(gtest_clusterer ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_color_utils  gtest_color_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_color_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_content_processing  gtest_content_processing.cpp
  ../src/image_utils/content_processing.h)
target_link_libraries(gtest_content_processing ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cv_conversion_float_uchar  gtest_cv_conversion_float_uchar.cpp)
TARGET_LINK_LIBRARIES(gtest_cv_conversion_float_uchar  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_drawing_utils
  gtest_drawing_utils.cpp
  ../src/image_utils/drawing_utils.h)
target_link_libraries(gtest_drawing_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ellipse_utils
  gtest_ellipse_utils.cpp
  ../src/image_utils/ellipse_utils.h)
TARGET_LINK_LIBRARIES(gtest_ellipse_utils string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_filename_prefix2imgs
  gtest_filename_prefix2imgs.cpp
  ../src/databases_io/filename_prefix2imgs.h
  ../src/databases_io/database_player.h
)
target_link_libraries(gtest_filename_prefix2imgs ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_google_images_search
  gtest_google_images_search.cpp
  ../src/image_utils/google_images_search.h
  TIMEOUT 300.0) # 3 minutes
TARGET_LINK_LIBRARIES(gtest_google_images_search string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_GraphMaker  gtest_GraphMaker.cpp)
TARGET_LINK_LIBRARIES(gtest_GraphMaker  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ground_plane_finder
  gtest_ground_plane_finder.cpp
  ../src/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_ground_plane_finder string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_head_finder
  gtest_head_finder.cpp
  ../src/image_utils/head_finder.h)
target_link_libraries(gtest_head_finder ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_histogram_utils
  gtest_histogram_utils.cpp
  ../src/visu_utils/histogram_utils.h)
target_link_libraries(gtest_histogram_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_clusterer
  gtest_image_clusterer.cpp
  ../src/point_clouds/image_clusterer.h)
target_link_libraries(gtest_image_clusterer ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_comparer
  gtest_image_comparer.cpp
  ../src/image_utils/image_comparer.h)
TARGET_LINK_LIBRARIES(gtest_image_comparer string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_contour
  gtest_image_contour.cpp
  ../src/image_utils/image_contour.h)
target_link_libraries(gtest_image_contour ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_utils  gtest_image_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_image_utils  ${catkin_LIBRARIES} disjoint_sets2)
catkin_add_gtest     (gtest_io
  gtest_io.cpp
  ../src/image_utils/io.h)
target_link_libraries(gtest_io ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_layer_utils
  gtest_layer_utils.cpp
  ../src/image_utils/layer_utils.h)
target_link_libraries(gtest_layer_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_mini_stage  gtest_mini_stage.cpp)
TARGET_LINK_LIBRARIES(gtest_mini_stage string_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_nano_skill
  gtest_nano_skill.cpp
  ../src/skill_templates/nano_skill.h)
target_link_libraries(gtest_nano_skill ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ocr_utils
  gtest_ocr_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_ocr_utils  OcrUtils translator ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_opencv_face_detector  gtest_opencv_face_detector.cpp)
TARGET_LINK_LIBRARIES(gtest_opencv_face_detector  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_opencv_ml  gtest_opencv_ml.cpp)
target_link_libraries(gtest_opencv_ml ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_pie_chart_utils  gtest_pie_chart_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_pie_chart_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_resize_utils
  gtest_resize_utils.cpp
  ../src/image_utils/resize_utils.h)
target_link_libraries(gtest_resize_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_timer_chart  gtest_timer_chart.cpp
    ../src/visu_utils/timer_chart.h)
TARGET_LINK_LIBRARIES(gtest_timer_chart  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_voronoi
  gtest_voronoi.cpp
  ../src/image_utils/image_contour.h
  ../src/image_utils/voronoi.h)
target_link_libraries(gtest_voronoi ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_xml_images_interface  gtest_xml_images_interface.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_interface  ${catkin_LIBRARIES} xml_images_interface)

catkin_add_gtest     (gtest_xml_images_reader  gtest_xml_images_reader.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_reader xml_images_reader ${catkin_LIBRARIES})
