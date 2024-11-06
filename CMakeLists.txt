cmake_minimum_required(VERSION 3.5)
project(qrb_ros_pipeline_manager)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Include directories
include_directories(include)

# Add the executable
ament_auto_add_executable(pipeline_manager
  src/pipeline_manager.cpp
  src/pipeline_server.cpp
)

ament_auto_add_executable(test_app
  test/test_app.cpp
)

# Install the executable
install(TARGETS pipeline_manager
  DESTINATION lib/${PROJECT_NAME})

# Export package
ament_auto_package()
