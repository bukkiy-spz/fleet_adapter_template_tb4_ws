cmake_minimum_required(VERSION 3.8)
project(tb4_fleet_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/fleet_adapter
    scripts/host_robot_topic_relay
    scripts/host_robot_topic_tcp_receiver
    scripts/host_robot_topic_tcp_sender
    scripts/zenoh_topics_to_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    config.yaml
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
