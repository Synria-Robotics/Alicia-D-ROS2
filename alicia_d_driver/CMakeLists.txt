cmake_minimum_required(VERSION 3.8)
project(alicia_d_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all necessary packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)  # <--- Add this

# 2. Use pkg-config to find libserial
pkg_check_modules(PC_SERIAL REQUIRED IMPORTED_TARGET libserial)

# No custom messages; skip rosidl generation

# 4. Build the reusable serial communicator library
add_library(serial_communicator_lib SHARED src/serial_communicator.cpp)
target_include_directories(serial_communicator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PC_SERIAL_INCLUDE_DIRS}
)
ament_target_dependencies(serial_communicator_lib
  rclcpp
  std_msgs
  sensor_msgs
  serial_driver
)
target_link_libraries(serial_communicator_lib
  PkgConfig::PC_SERIAL
)

# # 5. Build the ros2_control hardware interface as a SHARED LIBRARY
# add_library(alicia_hardware_interface SHARED src/alicia_hardware_interface.cpp)
# target_include_directories(alicia_hardware_interface PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(alicia_hardware_interface
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   std_msgs
#   sensor_msgs
#   serial_driver
# )
# target_link_libraries(alicia_hardware_interface
#   serial_communicator_lib
# )

# 6. Build the node executable
add_executable(alicia_d_driver_node src/alicia_d_driver_node.cpp)
target_include_directories(alicia_d_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(alicia_d_driver_node
  rclcpp
  std_msgs
  sensor_msgs
  serial_driver
)
target_link_libraries(alicia_d_driver_node
  serial_communicator_lib
)

# No rosidl target interfaces

# 8. Install targets and resources
install(TARGETS
  alicia_d_driver_node
  # alicia_hardware_interface
  serial_communicator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES alicia_d_driver.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# No msg directory to install

# pluginlib_export_plugin_description_file(hardware_interface alicia_d_driver.xml)

ament_package()