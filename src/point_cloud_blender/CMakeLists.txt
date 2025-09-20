cmake_minimum_required(VERSION 3.8)
project(point_cloud_blender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(arduinoWriter src/arduinoWriter.cpp)
ament_target_dependencies(arduinoWriter rclcpp std_msgs)
target_link_libraries(arduinoWriter
  Boost::system
  Boost::filesystem
  Boost::thread
)

add_executable(pointCloudBlender src/pointCloudBlender.cpp)
ament_target_dependencies(pointCloudBlender rclcpp std_msgs sensor_msgs message_filters pcl_conversions Eigen3)
target_link_libraries(pointCloudBlender
  ${PCL_LIBRARIES}
)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mergeNode src/mergeNode.cpp)
ament_target_dependencies(mergeNode rclcpp std_msgs sensor_msgs message_filters pcl_conversions Eigen3)
target_link_libraries(mergeNode
  ${PCL_LIBRARIES}
  Boost::system
  Boost::filesystem
  Boost::thread
)

install(TARGETS
  arduinoWriter
  pointCloudBlender
  mergeNode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
