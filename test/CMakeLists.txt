catkin_add_gtest     (gtest_array_to_color  gtest_array_to_color.cpp)
TARGET_LINK_LIBRARIES(gtest_array_to_color  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_atan3D gtest_atan3D.cpp )
target_link_libraries(gtest_atan3D  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_blob_segmenter gtest_blob_segmenter.cpp)
TARGET_LINK_LIBRARIES(gtest_blob_segmenter  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_border_remover  gtest_border_remover.cpp)
target_link_libraries(gtest_border_remover ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_tilter  gtest_cloud_tilter.cpp)
TARGET_LINK_LIBRARIES(gtest_cloud_tilter  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_viewer_gnuplot  gtest_cloud_viewer_gnuplot.cpp)
TARGET_LINK_LIBRARIES(gtest_cloud_viewer_gnuplot  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cloud_viewer  gtest_cloud_viewer.cpp)
target_link_libraries(gtest_cloud_viewer ${PCL_LIBRARIES} )

catkin_add_gtest     (gtest_clusterer_vision_utils  gtest_clusterer.cpp)
target_link_libraries(gtest_clusterer_vision_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_color_utils  gtest_color_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_color_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_content_processing  gtest_content_processing.cpp)
target_link_libraries(gtest_content_processing ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cv_conversion_float_uchar  gtest_cv_conversion_float_uchar.cpp)
TARGET_LINK_LIBRARIES(gtest_cv_conversion_float_uchar  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_drawing_utils  gtest_drawing_utils.cpp)
target_link_libraries(gtest_drawing_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_filename_prefix2imgs  gtest_filename_prefix2imgs.cpp)
target_link_libraries(gtest_filename_prefix2imgs ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_google_images_search gtest_google_images_search.cpp)
TARGET_LINK_LIBRARIES(gtest_google_images_search  ${catkin_LIBRARIES} curl)

catkin_add_gtest     (gtest_GraphMaker  gtest_GraphMaker.cpp)
TARGET_LINK_LIBRARIES(gtest_GraphMaker  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ground_plane_finder gtest_ground_plane_finder.cpp)
TARGET_LINK_LIBRARIES(gtest_ground_plane_finder  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_head_finder gtest_head_finder.cpp)
target_link_libraries(gtest_head_finder ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_histogram_utils gtest_histogram_utils.cpp)
target_link_libraries(gtest_histogram_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_clusterer  gtest_image_clusterer.cpp)
target_link_libraries(gtest_image_clusterer ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_comparer  gtest_image_comparer.cpp)
TARGET_LINK_LIBRARIES(gtest_image_comparer  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_contour  gtest_image_contour.cpp)
target_link_libraries(gtest_image_contour ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_image_utils  gtest_image_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_image_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_interpolator gtest_interpolator.cpp)
target_link_libraries(gtest_interpolator ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_io gtest_io.cpp)
target_link_libraries(gtest_io ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_kinect_openni_utils gtest_kinect_openni_utils.cpp )
target_link_libraries(gtest_kinect_openni_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_layer_utils  gtest_layer_utils.cpp)
target_link_libraries(gtest_layer_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_mask_acceleration  gtest_mask_acceleration.cpp)
TARGET_LINK_LIBRARIES(gtest_mask_acceleration ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_nano_skill  gtest_nano_skill.cpp)
target_link_libraries(gtest_nano_skill ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ocr_utils  gtest_ocr_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_ocr_utils   ${catkin_LIBRARIES} curl)

catkin_add_gtest     (gtest_opencv_face_detector  gtest_opencv_face_detector.cpp)
TARGET_LINK_LIBRARIES(gtest_opencv_face_detector  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_opencv_ml  gtest_opencv_ml.cpp)
target_link_libraries(gtest_opencv_ml ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_pie_chart_utils  gtest_pie_chart_utils.cpp)
TARGET_LINK_LIBRARIES(gtest_pie_chart_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_pplp_template gtest_pplp_template.cpp )
target_link_libraries(gtest_pplp_template  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ppl_utils gtest_ppl_utils.cpp )
target_link_libraries(gtest_ppl_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_ppl_viewer gtest_ppl_viewer.cpp  )
target_link_libraries(gtest_ppl_viewer  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_resize_utils  gtest_resize_utils.cpp)
target_link_libraries(gtest_resize_utils ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_timer_chart  gtest_timer_chart.cpp)
TARGET_LINK_LIBRARIES(gtest_timer_chart  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_voronoi  gtest_voronoi.cpp)
target_link_libraries(gtest_voronoi ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_string2msg  gtest_string2msg.cpp)
target_link_libraries(gtest_string2msg ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_xml_images_interface  gtest_xml_images_interface.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_interface  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_xml_images_reader  gtest_xml_images_reader.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_reader ${catkin_LIBRARIES})
