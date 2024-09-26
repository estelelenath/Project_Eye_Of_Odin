cmake_minimum_required(VERSION 3.5)
# cmake_minimum_required(VERSION 3.8) # if humble
project(project_eye_of_odin)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
