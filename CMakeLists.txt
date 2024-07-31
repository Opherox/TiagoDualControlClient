cmake_minimum_required(VERSION 3.0.2)
project(tiago_movement)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(move_tiago src/move_tiago.cpp)
target_link_libraries(move_tiago ${catkin_LIBRARIES})

## INSTALL ##

install(TARGETS move_tiago
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

