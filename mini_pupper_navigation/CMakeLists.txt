cmake_minimum_required(VERSION 3.5)
project(mini_pupper_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps param rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_localization_launch
    test/test_localization_launch.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_planning_launch
    test/test_planning_launch.py
    TIMEOUT 60
  )
endif()

ament_package()
