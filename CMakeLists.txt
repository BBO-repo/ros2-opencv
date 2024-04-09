cmake_minimum_required(VERSION 3.5)
project(ros2_opencv)

# default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# add node executables
add_executable(recognizer
    src/main.cpp
    src/data_processor/ImageDataProvider.cpp
    src/data_processor/Inference.cpp
    src/data_processor/Recognizer.cpp
)

# includes
include_directories(include/ros2_opencv)

ament_target_dependencies(recognizer rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

# install target
install(TARGETS recognizer DESTINATION lib/${PROJECT_NAME})

ament_package()
