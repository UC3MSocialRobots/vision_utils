
### rules for CompLabellerInterface - the interface
add_maggie_lib( comp_labeller_interface
    SOURCES     comp_labeller_interface.cpp
    LINK
)

### rules for the second implementation of disjoint sets
add_maggie_lib( disjoint_sets2
    SOURCES     disjoint_sets2.cpp
    LINK        opencv comp_labeller_interface
)

### rules for CBlobWrapper
add_maggie_lib( cblob_wrapper
    SOURCES     cblob_wrapper.cpp
    LINK        opencv comp_labeller_interface blob
)

### rules for CvBlobWrapper
add_maggie_lib( cv_blob_wrapper
    SOURCES     cv_blob_wrapper.cpp
    LINK        opencv comp_labeller_interface cvblob
)

### rules for CvFloodFillWrapper
add_maggie_lib( cv_flood_fill_wrapper
    SOURCES     cv_flood_fill_wrapper.cpp
    LINK        opencv comp_labeller_interface
)

### the tests
add_maggie_test(test_comp_labeller_interface.exe
    SOURCES     test_comp_labeller_interface.cpp
    LINK        xml_images_reader
                disjoint_sets2
                cblob_wrapper
                cv_blob_wrapper
                cv_flood_fill_wrapper
)

#add_maggie_lib( disjoint_sets
#    SOURCES     disjoint_sets.cpp
#)

#add_maggie_test(test_blobs.exe
#    SOURCES     test_blobs.cpp
#    LINK         cvblob blob
#)
