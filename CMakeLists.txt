cmake_minimum_required(VERSION 3.1.3)
project(ur5_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES tf
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/publisharucotfreal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/publisharucotfsimu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
