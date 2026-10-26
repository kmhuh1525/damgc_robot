cmake_minimum_required(VERSION 3.8)
project(rescue_robot_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/depth_to_csv.py
  scripts/dynamixel_orin.py
  scripts/dynamixel_orin_node.py
  scripts/gripper_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_grasp_task test/test_grasp_task.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_gripper_sequence test/test_gripper_sequence_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_dynamixel_orin_node test/test_dynamixel_orin_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(test_dynamixel_orin test/test_dynamixel_orin.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
