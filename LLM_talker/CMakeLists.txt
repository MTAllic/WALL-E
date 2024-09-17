cmake_minimum_required(VERSION 3.5)
project(llm_talker)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

