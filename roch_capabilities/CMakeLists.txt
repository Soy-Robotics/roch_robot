cmake_minimum_required(VERSION 2.8.3)
project(roch_capabilities)

find_package(catkin REQUIRED)

catkin_package()


install(DIRECTORY interfaces providers launch param
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

