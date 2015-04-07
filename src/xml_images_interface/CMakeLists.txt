add_library(xml_images_interface xml_images_interface.cpp)
target_link_libraries(xml_images_interface xml_images_reader ${catkin_LIBRARIES})

install(TARGETS xml_images_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
