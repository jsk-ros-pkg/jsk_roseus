cmake_minimum_required(VERSION 2.8.3)
project(eusgazebo)

catkin_package()

find_package(catkin REQUIRED COMPONENTS rostest)

## Install ##
install(DIRECTORY euslisp test scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

## Testing ##
#add_rostest(test/fall-arrow-object-simulation.test)
