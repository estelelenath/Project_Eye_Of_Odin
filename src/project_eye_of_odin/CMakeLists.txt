cmake_minimum_required(VERSION 3.5)
# cmake_minimum_required(VERSION 3.8) # if humble

# # cmake_minimum_required
# if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
#     cmake_minimum_required(VERSION 3.5)
#     message(STATUS "Building for Foxy - CMake 3.5, C++14")
# elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
#     cmake_minimum_required(VERSION 3.8)
#     message(STATUS "Building for Humble - CMake 3.8, C++17")
# else()
#     cmake_minimum_required(VERSION 3.5)  # Default Value, when ros2 version is unknown...
#     set(CMAKE_CXX_STANDARD 14)
#     message(WARNING "Unknown ROS distro, using default: CMake 3.5, C++14")
# endif()

project(project_eye_of_odin)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
