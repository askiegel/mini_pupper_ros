cmake_minimum_required(VERSION 3.5)
project(mini_pupper_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(
  odometry_nav_tf_relay
  src/odometry_nav_tf_relay.cpp
)

ament_target_dependencies(
  odometry_nav_tf_relay
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_msgs
)

install(
  TARGETS odometry_nav_tf_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/latest_tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees launch maps param rviz
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

  ament_add_pytest_test(
    test_guarded_navigation_launch
    test/test_guarded_navigation_launch.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_latest_tf_relay
    test/test_latest_tf_relay.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_odometry_nav_tf_relay
    test/test_odometry_nav_tf_relay.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_guarded_navigation_transform_tolerance
    test/test_guarded_navigation_transform_tolerance.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_guarded_navigation_controller_transform_tolerance
    test/test_guarded_navigation_controller_transform_tolerance.py
    TIMEOUT 60
  )
endif()

ament_package()
