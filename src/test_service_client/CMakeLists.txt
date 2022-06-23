cmake_minimum_required(VERSION 3.5)
project(test_service_client)

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  rclcpp
  example_interfaces
)

# Server

add_executable(service_server
  src/service_server.cpp
)

ament_target_dependencies(service_server
  ${dependencies}
)

install(TARGETS
  service_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Client

find_package(example_interfaces)

add_executable(service_client
  src/service_client.cpp
)

target_link_libraries(service_client
  ${library_name}
)

ament_target_dependencies(service_client
  ${dependencies}
)

install(TARGETS
  service_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()