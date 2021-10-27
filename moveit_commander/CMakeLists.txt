cmake_minimum_required(VERSION 3.1.3)
project(moveit_commander)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS bin/${PROJECT_NAME}_cmdline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(mv_l src/mv_l.py)
add_executable(mv_r src/mv_r.py)

add_subdirectory(test)
