cmake_minimum_required(VERSION 2.8.3)
project(roseus_smach)

find_package(catkin REQUIRED COMPONENTS euslisp roseus)

catkin_package(
    DEPENDS euslisp roseus # executive_smach
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY sample src test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
