cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher/member_function.cpp)
# add_executable(talker src/publisher/Lambda_member_function.cpp)
# add_executable(talker src/publisher/member_function_with_type_adapter.cpp)
# add_executable(talker src/publisher/member_function_with_unique_network_flow_endpoints.cpp)
# add_executable(talker src/publisher/member_function_with_wait_for_all_acked.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# add_executable(listener src/subscriber/member_function.cpp)
# add_executable(listener src/subscriber/Lambda_member_function.cpp)
add_executable(listener src/subscriber/member_function_with_type_adapter.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})  


ament_package()
