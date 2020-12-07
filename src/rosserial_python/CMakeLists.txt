cmake_minimum_required(VERSION 2.8.3)
project(rosserial_python)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS nodes/message_info_service.py nodes/serial_node.py nodes/serial_node_1.py nodes/serial_node_2.py nodes/serial_node_3.py nodes/serial_node_4.py nodes/serial_node_5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
