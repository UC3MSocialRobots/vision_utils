add_maggie_test(test_opencv_image_access.exe
    SOURCES     test_opencv_image_access.cpp
    LINK        opencv
)

add_maggie_test(test_opencv_image_resize.exe
    SOURCES     test_opencv_image_resize.cpp
    LINK        opencv
)

add_maggie_test(test_opencv_morphology_operators.exe
    SOURCES     test_opencv_morphology_operators.cpp
    LINK        opencv
)

add_maggie_test(test_opencv_text_fonts.exe
    SOURCES     test_opencv_text_fonts.cpp
    LINK        opencv
)

add_maggie_test(test_opencv_image_io_speed.exe
    SOURCES     test_opencv_image_io_speed.cpp
    LINK        opencv
)

