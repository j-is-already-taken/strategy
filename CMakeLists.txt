cmake_minimum_required(VERSION 3.8)
project(teamJ_software)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# LibTorchのパスを指定
#set(CMAKE_PREFIX_PATH /home/dan/pytorch)
#find_package(Torch REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

#--- for C++
include_directories(include)
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp std_msgs)

add_executable(sub_image src/sub_image.cpp)
ament_target_dependencies(sub_image rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(test_image src/test_image.cpp)
target_link_libraries(test_image "${TORCH_LIBRARIES}")
ament_target_dependencies(test_image rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(detect_yellow src/detect_yellow.cpp)
ament_target_dependencies(detect_yellow rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(serial_node src/serial_node.cpp)
ament_target_dependencies(serial_node rclcpp std_msgs)

add_executable(move_robot src/move_robot.cpp)
ament_target_dependencies(move_robot rclcpp geometry_msgs)

install(TARGETS
  cpp_node
  sub_image
  test_image
  detect_yellow
  serial_node
  move_robot
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/py_node.py
  scripts/sub_image.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter ...
  # ...
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
