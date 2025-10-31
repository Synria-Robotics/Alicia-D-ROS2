cmake_minimum_required(VERSION 3.8)
project(alicia_d_descriptions)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories (mirror structure of alicia_d_descriptions)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# Install Python launch files
install(PROGRAMS
  launch/display.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()