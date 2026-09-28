cmake_minimum_required(VERSION 3.8)
project(rescue_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/arrow_key_teleop.py
  scripts/costmap_topic_alias.py
  scripts/vslam_covariance_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_leader_apriltag_drive_launch
    test/test_leader_apriltag_drive_launch.py
  )
  ament_add_pytest_test(
    test_survivor_pipeline_launch
    test/test_survivor_pipeline_launch.py
  )
endif()

ament_package()
