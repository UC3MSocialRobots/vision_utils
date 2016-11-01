add_executable(oni2imgs.exe oni2imgs.cpp  )
target_link_libraries(oni2imgs.exe OpenNI ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(dgaitdb.exe dgaitdb.cpp  )
target_link_libraries(dgaitdb.exe OpenNI  ${catkin_LIBRARIES})

add_executable(rlpd2imgs.exe rlpd2imgs.cpp  )
target_link_libraries(rlpd2imgs.exe ${catkin_LIBRARIES})
