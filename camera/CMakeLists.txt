cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_node src/camera.cpp)

add_executable(image_subscriber src/subscriber.cpp)
ament_target_dependencies(
  image_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_node
  ${OpenCV_LIBS}
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
