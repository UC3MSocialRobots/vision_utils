add_executable(pplp_benchmarker.exe pplp_benchmarker.cpp)
target_link_libraries(pplp_benchmarker.exe lap_vision_utils ${catkin_LIBRARIES})
add_dependencies(pplp_benchmarker.exe ${catkin_EXPORTED_TARGETS})

add_executable(dgait2ppl.exe dgait2ppl.cpp)
target_link_libraries(dgait2ppl.exe disjoint_sets2 ${catkin_LIBRARIES})
add_dependencies(dgait2ppl.exe ${catkin_EXPORTED_TARGETS})

add_executable(KTP2PPL.exe KTP2PPL.cpp)
target_link_libraries(KTP2PPL.exe disjoint_sets2 ${catkin_LIBRARIES})
add_dependencies(KTP2PPL.exe ${catkin_EXPORTED_TARGETS})

add_executable(rlpd2ppl.exe rlpd2ppl.cpp
  ../../include/people_detection_vision/rlpd2imgs.h)
target_link_libraries(rlpd2ppl.exe ${catkin_LIBRARIES})
add_dependencies(rlpd2ppl.exe ${catkin_EXPORTED_TARGETS})
