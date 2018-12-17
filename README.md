roch_robot
===========

| Patform | Status |
|--------------------|----------------|
|amd64| [![Build Status](http://build.ros.org/job/Idoc__roch_robot__ubuntu_trusty_amd64/badge/icon)](http://build.ros.org/job/Idoc__roch_robot__ubuntu_trusty_amd64/)|
|armhf| [![Build Status](http://build.ros.org/view/Ibin_arm_uThf/job/Ibin_arm_uThf__roch_robot__ubuntu_trusty_armhf__binary/badge/icon)](http://build.ros.org/view/Ibin_arm_uThf/job/Ibin_arm_uThf__roch_robot__ubuntu_trusty_armhf__binary/)|

Robot ROS packages for the SawYer roch, for operating robot hardware.

 - roch_base : Hardware driver for communicating with the onboard MCU.
 - roch_ftdi : Udev rules files that how to identify /dev/roch.
 - roch_control : Control configuration
 - roch_description : Robot description (URDF)
 - roch_msgs : Message definitions
 - roch_safety_controller : Roch safety controller avoid collision.
 - roch_sensorpc : Roch sensors publisher using with navigation.
