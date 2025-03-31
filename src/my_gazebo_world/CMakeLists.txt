cmake_minimum_required(VERSION 3.0.2)
project(mp_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

catkin_package()

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/publish_camera_info.py
  scripts/point_cloud_segmentation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
