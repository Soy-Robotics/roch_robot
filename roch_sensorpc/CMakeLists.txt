cmake_minimum_required(VERSION 2.8.3)
project(roch_sensorpc)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet sensor_msgs roch_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp nodelet  sensor_msgs roch_msgs
   DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(roch_sensorpc_nodelet src/roch_sensorpc.cpp)
add_dependencies(roch_sensorpc_nodelet sensor_msgs_gencpp roch_msgs_gencpp)
target_link_libraries(roch_sensorpc_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS roch_sensorpc_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
