cmake_minimum_required(VERSION 3.8)
project(serial_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${geometry_msgs_INCLUDE_DIRS})


add_executable(serial_communication src/serial_communication.cpp)
target_link_libraries(serial_communication nlohmann_json::nlohmann_json)
ament_target_dependencies(serial_communication rclcpp std_msgs)
target_link_libraries(serial_communication ${Boost_LIBRARIES})
target_link_libraries(serial_communication ${geometry_msgs_LIBRARIES})

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

install(TARGETS
  serial_communication
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
