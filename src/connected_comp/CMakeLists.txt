### rules for the second implementation of disjoint sets
add_library(disjoint_sets2 disjoint_sets2.cpp)
target_link_libraries(disjoint_sets2 ${catkin_LIBRARIES} ${OpenCV_LIBS})
