add_executable(oni2imgs.exe oni2imgs.cpp oni2imgs.h database_player.h)
target_link_libraries(oni2imgs.exe OpenNI ${catkin_LIBRARIES})

add_executable(dgaitdb.exe dgaitdb.cpp dgaitdb.h database_player.h)
target_link_libraries(dgaitdb.exe OpenNI disjoint_sets2 string_utils ${catkin_LIBRARIES})

add_executable(rlpd2imgs.exe rlpd2imgs.cpp rlpd2imgs.h database_player.h)
target_link_libraries(rlpd2imgs.exe xml_document ${catkin_LIBRARIES})
