rosbuild_add_gtest(gtest_opencv_ml  gtest_opencv_ml.cpp)


rosbuild_add_gtest(gtest_mini_stage  gtest_mini_stage.cpp)
TARGET_LINK_LIBRARIES(gtest_mini_stage  string_utils)


rosbuild_add_gtest(gtest_xml_images_reader  gtest_xml_images_reader.cpp)
TARGET_LINK_LIBRARIES(gtest_xml_images_reader xml_images_reader)


rosbuild_add_gtest(gtest_content_processing  gtest_content_processing.cpp
  ../vision_utils/image_utils/content_processing.h)


rosbuild_add_gtest(gtest_image_contour
  gtest_image_contour.cpp
  ../vision_utils/image_utils/image_contour.h)


rosbuild_add_gtest(gtest_voronoi
  gtest_voronoi.cpp
  ../vision_utils/image_utils/image_contour.h
  ../vision_utils/image_utils/voronoi.h)


rosbuild_add_gtest(gtest_image_comparer
  gtest_image_comparer.cpp
  ../vision_utils/image_utils/image_comparer.h)
TARGET_LINK_LIBRARIES(gtest_image_comparer  string_utils)

rosbuild_add_gtest(gtest_drawing_utils
  gtest_drawing_utils.cpp
  ../vision_utils/image_utils/drawing_utils.h)


rosbuild_add_gtest(gtest_resize_utils
  gtest_resize_utils.cpp
  ../vision_utils/image_utils/resize_utils.h)


rosbuild_add_gtest(gtest_layer_utils
  gtest_layer_utils.cpp
  ../vision_utils/image_utils/layer_utils.h)

rosbuild_add_gtest(gtest_ellipse_utils
  gtest_ellipse_utils.cpp
  ../vision_utils/image_utils/ellipse_utils.h)
TARGET_LINK_LIBRARIES(gtest_ellipse_utils  string_utils)

rosbuild_add_gtest(gtest_histogram_utils
  gtest_histogram_utils.cpp
  ../vision_utils/visu_utils/histogram_utils.h)

rosbuild_add_gtest(gtest_google_images_search
  gtest_google_images_search.cpp
  ../vision_utils/image_utils/google_images_search.h
  TIMEOUT 300.0) # 3 minutes
TARGET_LINK_LIBRARIES(gtest_google_images_search  string_utils)


# point clouds stuff
rosbuild_add_gtest(gtest_ground_plane_finder
  gtest_ground_plane_finder.cpp
  ../vision_utils/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_ground_plane_finder  string_utils)

ROSBUILD_ADD_GTEST(gtest_clusterer
  gtest_clusterer.cpp
  ../vision_utils/point_clouds/clusterer.h)

ROSBUILD_ADD_GTEST(gtest_cloud_tilter
  gtest_cloud_tilter.cpp
  ../vision_utils/point_clouds/cloud_tilter.h)
TARGET_LINK_LIBRARIES(gtest_cloud_tilter  string_utils)

ROSBUILD_ADD_GTEST(gtest_cloud_viewer
  gtest_cloud_viewer.cpp
  ../vision_utils/point_clouds/cloud_viewer.h)
target_link_libraries(gtest_cloud_viewer ${PCL_LIBRARIES} )

ROSBUILD_ADD_GTEST(gtest_cloud_viewer_gnuplot
  gtest_cloud_viewer_gnuplot.cpp
  ../vision_utils/point_clouds/cloud_viewer_gnuplot.h)
TARGET_LINK_LIBRARIES(gtest_cloud_viewer_gnuplot string_utils)

rosbuild_add_gtest(gtest_blob_segmenter
  gtest_blob_segmenter.cpp
  ../vision_utils/point_clouds/blob_segmenter.h
  ../vision_utils/point_clouds/ground_plane_finder.h)
TARGET_LINK_LIBRARIES(gtest_blob_segmenter  string_utils disjoint_sets2)

rosbuild_add_gtest(gtest_image_clusterer
  gtest_image_clusterer.cpp
  ../vision_utils/point_clouds/image_clusterer.h
)

rosbuild_add_gtest(gtest_ocr_utils
  gtest_ocr_utils.cpp
)
TARGET_LINK_LIBRARIES(gtest_ocr_utils  OcrUtils translator)

#### database
rosbuild_add_gtest(gtest_nano_skill
  gtest_nano_skill.cpp
  ../vision_utils/skill_templates/nano_skill.h
)


#### database
rosbuild_add_gtest(gtest_filename_prefix2imgs
  gtest_filename_prefix2imgs.cpp
  ../vision_utils/databases_io/filename_prefix2imgs.h
  ../vision_utils/databases_io/database_player.h
)
rosbuild_add_gtest(gtest_liris2imgs
  gtest_liris2imgs.cpp
  ../vision_utils/databases_io/liris2imgs.h
  TIMEOUT 240.0) # 4 minutes
TARGET_LINK_LIBRARIES(gtest_liris2imgs  xml_document)

rosbuild_add_gtest(gtest_g3d2imgs
  gtest_g3d2imgs.cpp
  ../vision_utils/databases_io/g3d2imgs.h
)
TARGET_LINK_LIBRARIES(gtest_g3d2imgs  xml_document)
