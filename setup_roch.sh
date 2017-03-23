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
export ROCH_3D_SENSOR=astra  	#for now support three sensors [ asus_xtion_pro, kinect, r200, astra]
export ROCH_3D_SENSOR_ENABLE=false	#3d sensor enable
export ROCH_3D_SENSOR_NAV_ENABLE=false	#3d sensor enable with navigation
export ROCH_LASER=rplidar			#for now support two lasers [ls01c, rplidar]
export ROCH_LASER_ENABLE=true		#laser enable
export ROCH_SIMULATION=false

