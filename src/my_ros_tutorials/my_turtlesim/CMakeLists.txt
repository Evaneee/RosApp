cmake_minimum_required(VERSION 3.20)
project(my_turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(USE_QT6)
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 5)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(my_turtlesim_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

if(${QT_VERSION_MAJOR} GREATER "5")
  qt_standard_project_setup()
  qt_wrap_cpp(my_turtlesim_node_MOCS include/my_turtlesim/my_turtle_frame.hpp)
else()
  qt5_wrap_cpp(my_turtlesim_node_MOCS include/my_turtlesim/my_turtle_frame.hpp)
endif()

add_library(MocHeaders INTERFACE)
target_include_directories(MocHeaders INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(my_turtlesim_node
  src/my_turtlesim.cpp
  src/my_turtle.cpp
  src/my_turtle_frame.cpp
  src/agv_modbus_mapping.cpp    # <--- 新增这一行
  src/agv_state_machine.cpp      # <--- 新增这一行
  ${my_turtlesim_node_MOCS}
)

target_include_directories(my_turtlesim_node PRIVATE
  ${LIBMODBUS_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${my_turtlesim_msgs_DIR}/../../../include>
)

target_link_libraries(my_turtlesim_node PRIVATE
  ament_index_cpp::ament_index_cpp
  ${cpp_typesupport_target}
  ${geometry_msgs_TARGETS}
  ${my_turtlesim_msgs_TARGETS}
  Qt${QT_VERSION_MAJOR}::Widgets
  MocHeaders
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  ${LIBMODBUS_LIBRARIES}
)

add_executable(my_turtle_teleop_key tutorials/teleop_my_turtle_key.cpp)
target_link_libraries(my_turtle_teleop_key PRIVATE
  ${my_turtlesim_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  MocHeaders
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square PRIVATE
  ${my_turtlesim_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  MocHeaders
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic PRIVATE
  ${my_turtlesim_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  MocHeaders
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  my_turtlesim_node
  my_turtle_teleop_key
  draw_square
  mimic
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
