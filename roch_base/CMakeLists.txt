cmake_minimum_required(VERSION 2.8.3)
project(roch_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface roch_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES core
  CATKIN_DEPENDS diagnostic_updater hardware_interface roch_msgs roscpp sensor_msgs geometry_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(core
  src/core/crc.cpp
  src/core/Logger.cpp
  src/core/Message.cpp
  src/core/Message_data.cpp
  src/core/Message_request.cpp
  src/core/Message_cmd.cpp
  src/core/Transport.cpp
  src/core/Number.cpp
  src/core/linux_serial.cpp
)

add_executable(roch_node
  src/roch_base.cpp
  src/roch_hardware.cpp
  src/roch_diagnostics.cpp
  src/core_wrapper.cpp)
target_link_libraries(roch_node core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roch_node roch_msgs_gencpp)

add_executable(test_encoder 
  test/test_encoder.cpp
  src/core_wrapper.cpp)
target_link_libraries(test_encoder core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_imu
  test/test_imu.cpp
  src/core_wrapper.cpp)
target_link_libraries(test_imu core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_speed
  test/test_speed.cpp
  src/core_wrapper.cpp)
target_link_libraries(test_speed core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_rangefinder
  test/test_rangefinder.cpp
  src/core_wrapper.cpp)
target_link_libraries(test_rangefinder core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(test_encoder roch_msgs_gencpp)
roslint_cpp(
  src/roch_base.cpp
  src/roch_hardware.cpp
  src/roch_diagnostics.cpp
  include/roch_base/roch_diagnostics.h
  include/roch_base/roch_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS core roch_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
