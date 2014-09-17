
### opencv_face_detector
add_maggie_test(test_opencv_face_detector.exe
    SOURCES     test_opencv_face_detector.cpp opencv_face_detector.h
    LINK        opencv
)


### border_remover
add_maggie_test(test_border_remover.exe
    SOURCES     test_border_remover.cpp border_remover.h
    LINK        opencv
)

###
add_maggie_test(test_google_images_search.exe
    SOURCES     test_google_images_search.cpp google_images_search.h
    LINK        opencv string_utils
)

###
add_maggie_test(test_image_utils.exe
    SOURCES     test_image_utils.cpp
                layer_utils.h
                content_processing.h connected_comp_interface.h
                value_remover.h cv_border_closer.h  titlemaps.h
    LINK        disjoint_sets2 string_utils
)

add_maggie_test(test_drawing_utils.exe
    SOURCES     test_drawing_utils.cpp
                drawing_utils.h
    LINK
)

add_maggie_test(test_io.exe
    SOURCES     test_io.cpp
                io.h
    LINK
)
# MCP tests
add_maggie_test(test_voronoi.exe
    SOURCES     test_voronoi.cpp
                voronoi.h
    LINK
)
