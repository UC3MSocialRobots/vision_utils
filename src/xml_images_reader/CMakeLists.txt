add_library(xml_images_reader xml_images_reader.cpp)
target_link_libraries(xml_images_reader  ${catkin_LIBRARIES})

install(TARGETS xml_images_reader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
