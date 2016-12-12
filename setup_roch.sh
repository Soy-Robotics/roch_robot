# This configures the environment variables for a create based Roch
# running the Roch 2.0 software. This is necessary to run after
# setup.bash to ensure the create drivers and nodes are all correctly launched.
#
# You may wish to set the 3d sensor to asus_xtion_pro if you do not have a kinect
# though. While the kinect settings work for the asus in terms of 3d sensing (openni
# handles the abstraction) the asus settiing makes sure the mesh shown in rviz/gazebo 
# is the asus.

export ROCH_BASE=roch			#for now only have one [roch]
export ROCH_STACKS=standard		#for now only have one [standard]
export ROCH_3D_SENSOR=asus_xtion_pro	#for now support two sensor [ asus_xtion_pro, kinect]
export ROCH_3D_SENSOR_ENABLE=true	#3d sensor enable
export ROCH_LASER=ls01c			#for now only support ls01c
export ROCH_LASER_ENABLE=false		#laser enable
export ROCH_SIMULATION=false

