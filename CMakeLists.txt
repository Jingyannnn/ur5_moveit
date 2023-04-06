cmake_minimum_required(VERSION 3.1.3)
project(ur5_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
)

add_message_files(
  FILES
  Poses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES tf
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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


# add_executable(my_tf_listener src/getcambase.cpp)
# target_link_libraries(my_tf_listener ${catkin_LIBRARIES})

add_executable(sampleforrange src/sampleforrange.cpp)
target_link_libraries(sampleforrange ${catkin_LIBRARIES})
