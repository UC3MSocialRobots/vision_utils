IF(AD_USE_OPENCV)
  add_maggie_test(opencv_calibration
    SOURCES       opencv_calibration.cpp
  )

  IF(AD_USE_ROS)
    add_maggie_test(image_topic_to_video
      SOURCES       image_topic_to_video.cpp
      LINK          string_utils
    )

    add_maggie_test(rgb_and_depth_to_yaml
      SOURCES       rgb_and_depth_to_yaml.cpp
      LINK          string_utils
    )

    add_maggie_test(ros_camera_info_to_binary
      SOURCES       ros_camera_info_to_binary.cpp
      LINK          string_utils
    )
  ENDIF(AD_USE_ROS)

  add_maggie_test(launcher_camera_blocker_skill.exe
      SOURCES     launcher_camera_blocker_skill.cpp
      LINK        string_utils # non_verbal
  )

  add_maggie_lib (camera_utils
    SOURCES       camera_utils.cpp
    LINK          opencv
  )

  add_maggie_test(test_camera_utils.exe
    SOURCES       test_camera_utils.cpp
    LINK          camera_utils # camera_calibrator
  )
ENDIF(AD_USE_OPENCV)
