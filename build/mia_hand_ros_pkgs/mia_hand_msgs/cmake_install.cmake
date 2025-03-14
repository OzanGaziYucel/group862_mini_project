# Install script for directory: /home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ozan/group862_mini_project/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs/msg" TYPE FILE FILES
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs/srv" TYPE FILE FILES
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
    "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs/cmake" TYPE FILE FILES "/home/ozan/group862_mini_project/build/mia_hand_ros_pkgs/mia_hand_msgs/catkin_generated/installspace/mia_hand_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ozan/group862_mini_project/devel/include/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ozan/group862_mini_project/devel/share/roseus/ros/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ozan/group862_mini_project/devel/share/common-lisp/ros/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ozan/group862_mini_project/devel/share/gennodejs/ros/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ozan/group862_mini_project/devel/lib/python3/dist-packages/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ozan/group862_mini_project/devel/lib/python3/dist-packages/mia_hand_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ozan/group862_mini_project/build/mia_hand_ros_pkgs/mia_hand_msgs/catkin_generated/installspace/mia_hand_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs/cmake" TYPE FILE FILES "/home/ozan/group862_mini_project/build/mia_hand_ros_pkgs/mia_hand_msgs/catkin_generated/installspace/mia_hand_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs/cmake" TYPE FILE FILES
    "/home/ozan/group862_mini_project/build/mia_hand_ros_pkgs/mia_hand_msgs/catkin_generated/installspace/mia_hand_msgsConfig.cmake"
    "/home/ozan/group862_mini_project/build/mia_hand_ros_pkgs/mia_hand_msgs/catkin_generated/installspace/mia_hand_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mia_hand_msgs" TYPE FILE FILES "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/package.xml")
endif()

