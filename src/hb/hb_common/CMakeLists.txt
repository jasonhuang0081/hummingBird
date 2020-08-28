cmake_minimum_required(VERSION 2.8.3)
project(hb_common)

find_package(catkin REQUIRED COMPONENTS
  rospy
  hb_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy hb_msgs
)

# catkin_install_python(PROGRAMS
#   scripts/my_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
