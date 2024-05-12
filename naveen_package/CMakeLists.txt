cmake_minimum_required(VERSION 3.8)
project(naveen_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
script/order_complete.py
script/perform_kitting.py
script/order_kitting.py
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  resources
  meshes
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
