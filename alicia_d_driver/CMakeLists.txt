cmake_minimum_required(VERSION 3.16)
project(alicia_d_driver LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

set(NODE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${NODE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Use pkg-config to find libserial
pkg_check_modules(PC_SERIAL REQUIRED IMPORTED_TARGET libserial)

# Find zlib for CRC-32 calculation (matching PyCRC.CRC32)
find_package(ZLIB REQUIRED)

## COMPILE
# Build the reusable serial communicator library
add_library(serial_communicator_lib SHARED src/serial_communicator.cpp)
target_compile_features(serial_communicator_lib PUBLIC cxx_std_17)
target_include_directories(serial_communicator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/alicia_d_driver>
  ${PC_SERIAL_INCLUDE_DIRS}
)
ament_target_dependencies(serial_communicator_lib PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(serial_communicator_lib PUBLIC
  PkgConfig::PC_SERIAL
  ZLIB::ZLIB
)

# Build the ros2_control hardware interface as a SHARED LIBRARY
add_library(alicia_d_hardware_interface SHARED src/alicia_d_hardware_interface.cpp)
target_compile_features(alicia_d_hardware_interface PUBLIC cxx_std_17)
target_include_directories(alicia_d_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/alicia_d_driver>
)
ament_target_dependencies(alicia_d_hardware_interface PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)
target_link_libraries(alicia_d_hardware_interface PUBLIC
  serial_communicator_lib
  alicia_d_data_parser_control_lib
)

# Build the data parser control library
add_library(alicia_d_data_parser_control_lib SHARED src/alicia_d_data_parser_control.cpp)
target_compile_features(alicia_d_data_parser_control_lib PUBLIC cxx_std_17)
target_include_directories(alicia_d_data_parser_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/alicia_d_driver>
)
ament_target_dependencies(alicia_d_data_parser_control_lib PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(alicia_d_data_parser_control_lib PUBLIC
  serial_communicator_lib
)

# Build the standalone node executable
add_executable(alicia_d_driver_node src/alicia_d_driver_node.cpp)
target_compile_features(alicia_d_driver_node PUBLIC cxx_std_17)
target_include_directories(alicia_d_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/alicia_d_driver>
)
ament_target_dependencies(alicia_d_driver_node PUBLIC
  ${NODE_INCLUDE_DEPENDS}
)
target_link_libraries(alicia_d_driver_node PUBLIC
  serial_communicator_lib
  alicia_d_data_parser_control_lib
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface alicia_d_driver.xml)

## INSTALL
install(
  DIRECTORY include/
  DESTINATION include/alicia_d_driver
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES alicia_d_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS alicia_d_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS alicia_d_hardware_interface serial_communicator_lib alicia_d_data_parser_control_lib
  EXPORT export_alicia_d_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_alicia_d_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${NODE_INCLUDE_DEPENDS})
ament_package()