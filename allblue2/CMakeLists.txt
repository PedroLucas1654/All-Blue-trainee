cmake_minimum_required(VERSION 3.8)
project(allblue2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TestMotor.action"
  "action/TestAllMotors.action"
  DEPENDENCIES action_msgs
)

ament_python_install_package(allblue2_py)

install(PROGRAMS
  scripts/motor_controller
  scripts/motor_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
