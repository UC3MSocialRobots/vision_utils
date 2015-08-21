add_library(string_utils StringUtils.cpp StringUtils.cpp)
target_link_libraries(string_utils curl ${catkin_LIBRARIES})

add_library(lap lap.cpp)
