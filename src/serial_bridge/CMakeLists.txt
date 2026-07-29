cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  launch/serial_bridge_launch.py
  launch/real_robot_navigation_launch.py
  launch/real_robot_mapping_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
