cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Mission.action"
)

install(PROGRAMS
  scripts/mission_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_server
)

install(PROGRAMS
  scripts/mission_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_client
)

install(DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
