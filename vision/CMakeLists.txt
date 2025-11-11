cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # <-- Added OpenCV dependency

# --- Executable ---
add_executable(line_detection
  src/linedetection.cpp
  src/hough.cpp
)

# --- Include directory for your headers ---
target_include_directories(line_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}  # <-- Include OpenCV headers
)

# --- Link ROS2 dependencies ---
ament_target_dependencies(line_detection
  rclcpp
  std_msgs
)

# --- Link OpenCV libraries ---
target_link_libraries(line_detection
  ${OpenCV_LIBS}  # <-- Link OpenCV libraries
  uuid
)

# --- Installation ---
install(TARGETS
  line_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
