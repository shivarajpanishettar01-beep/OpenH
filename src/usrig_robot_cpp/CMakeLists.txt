cmake_minimum_required(VERSION 3.8)
project(usrig_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ur_client_library QUIET)

if(ur_client_library_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(usrig_interfaces REQUIRED)
  add_executable(robot_state_cpp src/robot_state_cpp.cpp)
  target_link_libraries(robot_state_cpp ur_client_library::urcl pthread)
  ament_target_dependencies(robot_state_cpp rclcpp std_msgs sensor_msgs geometry_msgs usrig_interfaces)
  install(TARGETS robot_state_cpp DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "ur_client_library not found: robot_state_cpp is NOT built "
                  "(sudo apt install ros-humble-ur-client-library). robot.launch.py falls back to "
                  "the Python robot_state.")
endif()

ament_package()
