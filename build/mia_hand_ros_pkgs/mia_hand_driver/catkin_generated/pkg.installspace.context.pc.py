# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;mia_hand_msgs;std_srvs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmia_hand_driver;-lserial".split(';') if "-lmia_hand_driver;-lserial" != "" else []
PROJECT_NAME = "mia_hand_driver"
PROJECT_SPACE_DIR = "/home/ozan/group862_mini_project/install"
PROJECT_VERSION = "1.0.2"
