# CMakeLists.txt（主要部）
cmake_minimum_required(VERSION 3.8)
project(motion_primitives)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(in_place_turn_node_lib src/in_place_turn_node.cpp)
target_include_directories(in_place_turn_node_lib PUBLIC include)
ament_target_dependencies(in_place_turn_node_lib rclcpp rclcpp_lifecycle geometry_msgs)

add_executable(in_place_turn src/in_place_turn.cpp)
target_link_libraries(in_place_turn in_place_turn_node_lib)
ament_target_dependencies(in_place_turn rclcpp rclcpp_lifecycle geometry_msgs)

add_library(stop_motion_node_lib src/stop_motion_node.cpp)
target_include_directories(stop_motion_node_lib PUBLIC include)
ament_target_dependencies(stop_motion_node_lib rclcpp rclcpp_lifecycle geometry_msgs)

add_executable(stop_motion src/stop_motion.cpp)
target_link_libraries(stop_motion stop_motion_node_lib)
ament_target_dependencies(stop_motion rclcpp rclcpp_lifecycle geometry_msgs)

install(TARGETS
  in_place_turn
  stop_motion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
