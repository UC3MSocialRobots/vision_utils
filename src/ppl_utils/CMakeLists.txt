add_executable(pose2ppl.exe
  pose2ppl.cpp
)
target_link_libraries(pose2ppl.exe
  ${catkin_LIBRARIES})
add_dependencies(pose2ppl.exe ${people_msgs_rl_EXPORTED_TARGETS})

add_executable(ppl2marker.exe
    ppl2marker.cpp
  ../../include/vision_utils/utils/geometry_utils.h
  ../../include/vision_utils/utils/marker_utils.h
  ../../include/vision_utils/utils/pt_utils.h
)
target_link_libraries(ppl2marker.exe ${catkin_LIBRARIES})
add_dependencies(ppl2marker.exe ${people_msgs_rl_EXPORTED_TARGETS})

add_executable(ppl_viewer.exe
  ppl_viewer.cpp
  ../../include/vision_utils/ppl_viewer.h
  ../../include/vision_utils/ppl2ms.h
  ../../include/vision_utils/mini_stage_plugins.h
)
target_link_libraries(ppl_viewer.exe
  ${catkin_LIBRARIES})
add_dependencies(ppl_viewer.exe ${people_msgs_rl_EXPORTED_TARGETS})
