cmake_minimum_required(VERSION 3.8)
project(turtle_multi_target)

# Стандарты
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_ros REQUIRED)

# Сообщения
set(msg_files
  "msg/CurrentTarget.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Python-пакет с кодом нод (название отличаем от имени ROS-пакета)
ament_python_install_package(turtle_multi_target_py)

# Установка исполняемых Python-нод
install(PROGRAMS
  turtle_multi_target_py/turtle_tf_broadcaster.py
  turtle_multi_target_py/target_switcher.py
  turtle_multi_target_py/turtle_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch-файлы
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
