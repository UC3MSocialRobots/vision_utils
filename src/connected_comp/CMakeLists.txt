
### rules for CompLabellerInterface - the interface
add_library(comp_labeller_interface comp_labeller_interface.cpp)

### rules for the second implementation of disjoint sets
add_library(disjoint_sets2 disjoint_sets2.cpp)
target_link_libraries(disjoint_sets2 ${catkin_LIBRARIES} comp_labeller_interface)

### rules for CBlobWrapper
add_library(cblob_wrapper cblob_wrapper.cpp)
target_link_libraries(cblob_wrapper ${catkin_LIBRARIES} comp_labeller_interface blob)

### rules for CvBlobWrapper
add_library(cv_blob_wrapper cv_blob_wrapper.cpp)
target_link_libraries(cv_blob_wrapper ${catkin_LIBRARIES} comp_labeller_interface cvblob)

### rules for CvFloodFillWrapper
add_library(cv_flood_fill_wrapper cv_flood_fill_wrapper.cpp)
target_link_libraries(cv_flood_fill_wrapper ${catkin_LIBRARIES} comp_labeller_interface)

### the tests
add_executable(test_comp_labeller_interface.exe test_comp_labeller_interface.cpp)
target_link_libraries(test_comp_labeller_interface.exe xml_images_reader
														disjoint_sets2
														cblob_wrapper
														cv_blob_wrapper
														cv_flood_fill_wrapper)

install(TARGETS comp_labeller_interface disjoint_sets2
                cblob_wrapper cv_blob_wrapper cv_flood_fill_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



