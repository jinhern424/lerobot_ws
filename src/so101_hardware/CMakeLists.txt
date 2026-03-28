cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Create the shared library from your C++ file
# Create the shared library from your C++ file AND the SDK files
add_library(${PROJECT_NAME} SHARED
  src/sts3215_hardware_interface.cpp
  src/SCS.cpp
  src/SCSerial.cpp
  src/SMS_STS.cpp
  src/SCSCL.cpp
  src/HLSCL.cpp
)

# Tell the compiler where to find your header (.hpp) files
# Tell the compiler where to find your header (.hpp and .h) files
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/so101_hardware> 
  $<INSTALL_INTERFACE:include>
)

# Link the ROS 2 dependencies to your library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# Export the plugin XML file so ROS 2 can find it
pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

# Install rules
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export for other packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib)

ament_package()