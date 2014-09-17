IF(AD_USE_OPENCV)
add_maggie_lib( xml_images_reader
    SOURCES     xml_images_reader.cpp
    LINK        xml_document opencv
)

add_maggie_test(test_xml_images_reader.exe
    SOURCES     test_xml_images_reader.cpp
    LINK        xml_images_reader
)
ENDIF(AD_USE_OPENCV)
