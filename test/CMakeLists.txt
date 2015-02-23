catkin_add_gtest(gtest_opencv_ml  gtest_opencv_ml.cpp)


catkin_add_gtest(gtest_mini_stage  gtest_mini_stage.cpp)
TARGET_LINK_LIBRARIES(gtest_mini_stage string_utils ${catkin_LIBRARIES})


catkin_add_gtest(gtest_xml_images_reader  gtest_xml_images_reader.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_reader xml_images_reader)


catkin_add_gtest(gtest_content_processing  gtest_content_processing.cpp
  ../src/image_utils/content_processing.h)


catkin_add_gtest(gtest_image_contour
  gtest_image_contour.cpp
  ../src/image_utils/image_contour.h)


catkin_add_gtest(gtest_voronoi
  gtest_voronoi.cpp
  ../src/image_utils/image_contour.h
  ../src/image_utils/voronoi.h)


catkin_add_gtest(gtest_image_comparer
  gtest_image_comparer.cpp
  ../src/image_utils/image_comparer.h)
TARGET_LINK_LIBRARIES(gtest_image_comparer string_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_drawing_utils
  gtest_drawing_utils.cpp
  ../src/image_utils/drawing_utils.h)


catkin_add_gtest(gtest_resize_utils
  gtest_resize_utils.cpp
  ../src/image_utils/resize_utils.h)


catkin_add_gtest(gtest_layer_utils
  gtest_layer_utils.cpp
  ../src/image_utils/layer_utils.h)

catkin_add_gtest(gtest_ellipse_utils
  gtest_ellipse_utils.cpp
  ../src/image_utils/ellipse_utils.h)
TARGET_LINK_LIBRARIES(gtest_ellipse_utils string_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_histogram_utils
  gtest_histogram_utils.cpp
  ../src/visu_utils/histogram_utils.h)

catkin_add_gtest(gtest_google_images_search
  gtest_google_images_search.cpp
  ../src/image_utils/google_images_search.h
  TIMEOUT 300.0) # 3 minutes
TARGET_LINK_LIBRARIES(gtest_google_images_search string_utils ${catkin_LIBRARIES})


# point clouds stuff
catkin_add_gtest(gtest_ground_plane_finder
  gtest_ground_plane_finder.cpp
  ../src/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_ground_plane_finder string_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_clusterer
  gtest_clusterer.cpp
  ../src/point_clouds/clusterer.h)

catkin_add_gtest(gtest_cloud_tilter
  gtest_cloud_tilter.cpp
  ../src/point_clouds/cloud_tilter.h)
TARGET_LINK_LIBRARIES(gtest_cloud_tilter string_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_cloud_viewer
  gtest_cloud_viewer.cpp
  ../src/point_clouds/cloud_viewer.h)
target_link_libraries(gtest_cloud_viewer ${PCL_LIBRARIES} )

catkin_add_gtest(gtest_cloud_viewer_gnuplot
  gtest_cloud_viewer_gnuplot.cpp
  ../src/point_clouds/cloud_viewer_gnuplot.h)
TARGET_LINK_LIBRARIES(gtest_cloud_viewer_gnuplot string_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_blob_segmenter
  gtest_blob_segmenter.cpp
  ../src/point_clouds/blob_segmenter.h
  ../src/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_blob_segmenter string_utils disjoint_sets2 ${catkin_LIBRARIES})

catkin_add_gtest(gtest_image_clusterer
  gtest_image_clusterer.cpp
  ../src/point_clouds/image_clusterer.h
)

catkin_add_gtest(gtest_ocr_utils
  gtest_ocr_utils.cpp
)
TARGET_LINK_LIBRARIES(gtest_ocr_utils  OcrUtils translator)

catkin_add_gtest(gtest_head_finder
  gtest_head_finder.cpp
  ../src/image_utils/head_finder.h)

### border_remover
catkin_add_gtest(gtest_border_remover
  gtest_border_remover.cpp
  ../src/image_utils/border_remover.h
)

catkin_add_gtest(gtest_io
  gtest_io.cpp
  ../src/image_utils/io.h
)


#### database
catkin_add_gtest(gtest_nano_skill
  gtest_nano_skill.cpp
  ../src/skill_templates/nano_skill.h
)


#### database
catkin_add_gtest(gtest_filename_prefix2imgs
  gtest_filename_prefix2imgs.cpp
  ../src/databases_io/filename_prefix2imgs.h
  ../src/databases_io/database_player.h
)
catkin_add_gtest(gtest_liris2imgs
  gtest_liris2imgs.cpp
  ../src/databases_io/liris2imgs.h
  TIMEOUT 240.0) # 4 minutes
TARGET_LINK_LIBRARIES(gtest_liris2imgs  xml_document)

catkin_add_gtest(gtest_g3d2imgs
  gtest_g3d2imgs.cpp
  ../src/databases_io/g3d2imgs.h
)
TARGET_LINK_LIBRARIES(gtest_g3d2imgs  xml_document)
