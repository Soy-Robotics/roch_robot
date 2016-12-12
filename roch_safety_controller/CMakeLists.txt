cmake_minimum_required(VERSION 2.8.3)
project(roch_safety_controller)
find_package(catkin REQUIRED COMPONENTS roscpp  std_msgs geometry_msgs roch_msgs yocs_controllers ecl_threads)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp  std_msgs geometry_msgs roch_msgs yocs_controllers ecl_threads
   DEPENDS Boost
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_executable(roch_safety_controller_nodelet src/nodelet.cpp)
add_dependencies(roch_safety_controller_nodelet roch_msgs_gencpp)
target_link_libraries(roch_safety_controller_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS roch_safety_controller_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
