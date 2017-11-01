cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_cc2017_car)

find_package(catkin)


catkin_package(
    CATKIN_DEPENDS
)


install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)
