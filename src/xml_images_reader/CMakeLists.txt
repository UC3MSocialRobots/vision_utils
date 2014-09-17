IF(AD_USE_OPENCV)
	add_library(xml_images_reader xml_images_reader.cpp)
        target_link_libraries(xml_images_reader xml_document ${catkin_LIBRARIES})

	add_executable(test_xml_images_reader.exe test_xml_images_reader.cpp)
	target_link_libraries(test_xml_images_reader.exe xml_images_reader)

	install(TARGETS xml_images_reader
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
ENDIF(AD_USE_OPENCV)
