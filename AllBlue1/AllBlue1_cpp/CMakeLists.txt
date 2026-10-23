cmake_minimum_required(VERSION 3.14)
project(AllBlue1_cpp LANGUAGES CXX)   # igual ao <name> do package.xml

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)   # main.cpp usa std::thread

# Núcleo sem dependência de ROS: parser NMEA (core.cpp) + serial POSIX (AllBlue1.cpp)
add_library(allblue1_core
  src/core.cpp
  src/AllBlue1.cpp)
target_include_directories(allblue1_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(allblue1_core PRIVATE -Wall -Wextra)

# Nó ROS 2
# ATENÇÃO: sem PRIVATE/PUBLIC aqui. O ament_target_dependencies (Humble) usa a forma
# simples de target_link_libraries, e o CMake não deixa misturar as duas no mesmo alvo.
add_executable(gps_node src/main.cpp)
target_link_libraries(gps_node allblue1_core Threads::Threads)
ament_target_dependencies(gps_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS allblue1_core gps_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_core tests/test_core.cpp)
  target_link_libraries(test_core PRIVATE allblue1_core)
  add_test(NAME test_core COMMAND test_core)
endif()

ament_package()
