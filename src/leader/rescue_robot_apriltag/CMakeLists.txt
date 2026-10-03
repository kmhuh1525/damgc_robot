cmake_minimum_required(VERSION 3.8)
project(rescue_robot_apriltag)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME}/docs)
install(PROGRAMS
  scripts/camera_info_qos_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  rescue_robot_apriltag/apriltag_approach_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apriltag_approach_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_approach_logic test/test_approach_logic.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_base_pose test/test_base_pose.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_base_alignment_logic test/test_base_alignment_logic.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_post_align_odom test/test_post_align_odom.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
