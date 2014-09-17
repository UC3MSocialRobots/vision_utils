add_maggie_lib( xml_images_interface
    SOURCES     xml_images_interface.cpp
    LINK        xml_images_reader
)

add_maggie_test(test_xml_images_interface.exe
    SOURCES     test_xml_images_interface.cpp
    LINK        xml_images_interface
)
