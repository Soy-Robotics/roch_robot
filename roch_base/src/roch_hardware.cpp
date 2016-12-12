/**
*
*  \author     Paul Bovbel <pbovbel@sawyerrobotics.com>
*  \copyright  Copyright (c) 2014-2015, sawyer Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of sawyer Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL sawyer ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@sawyerrobotics.com
*
*/

#include "roch_base/roch_hardware.h"
#include <boost/assign/list_of.hpp>
#include <float.h>


namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};
namespace roch_base
{

  /**
  * Initialize roch hardware
  */
  rochHardware::rochHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh),
    system_status_task_(roch_status_msg_),
    power_status_task_(roch_status_msg_),
    safety_status_task_(roch_status_msg_),
    software_status_task_(roch_status_msg_, target_control_freq)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.095);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 0.45);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 20.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/ttyUSB0");

    core::connect(port);
    core::configureLimits(max_speed_, max_accel_);
    resetTravelOffset();
    initializeDiagnostics(); 
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  * 得到当前MCU的旅行补偿和相对未来的编码器偏差系数
  */
  void rochHardware::resetTravelOffset()
  {
    core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      polling_timeout_);
      publishRawData();
    if (enc)
    {
		for (int i = 0; i < 2; i++)//新本 双驱
      {
        joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
#ifdef DEBUG_INFO 
	ROS_INFO("joints_[%d].position_offset:%.2lf  ",i,joints_[i].position_offset);
#endif
      }
    }
    else
    {
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
    
    //add imu data from IMU
   core::Channel<sawyer::Data6AxisYaw>::Ptr imuRateData = core::Channel<sawyer::Data6AxisYaw>::requestData(
     polling_timeout_);
     publishRawData();
     if(imuRateData){
      ROS_DEBUG_STREAM("Received  imu rate data information (Angle:" << imuRateData->getAngle() << " Angle rate:" << imuRateData->getAngleRate() << ")");
#ifdef DEBUG_INFO 
    ROS_INFO("Received  imu rate data information, angle_offset:%.2lf",imuRateData->getAngle());    
#endif 
      
      sixGyro.angle_offset = imuRateData->getAngle();
    }
    else{
       ROS_ERROR("Could not get imu data to calibrate rate offset");
    }
    core::Channel<sawyer::DataPlatformAcceleration>::Ptr getPlatformAccData = 
      core::Channel<sawyer::DataPlatformAcceleration>::requestData(polling_timeout_);
    publishRawData();
    if(getPlatformAccData){
#ifdef DEBUG_INFO 
    ROS_INFO("Received acc_x:%.2lf, acc_y:%.2lf, acc_z:%.2lf.",getPlatformAccData->getX(),getPlatformAccData->getY(),getPlatformAccData->getZ());
#endif 
      sixGyro.acc.X_Offset = getPlatformAccData->getX();
      sixGyro.acc.Y_Offset = getPlatformAccData->getY();
      sixGyro.acc.Z_Offset = getPlatformAccData->getZ();
      
      linear_acceleration[0] = sixGyro.acc.X_Offset;
      linear_acceleration[1] = sixGyro.acc.Y_Offset;
      linear_acceleration[2] = sixGyro.acc.Z_Offset;
    }
    else{
       ROS_ERROR("Could not get Gyro data to calibrate Acceleration offset");
    }
  }
void rochHardware::getRangefinderData()
{
  core::Channel<sawyer::DataRangefinders>::Ptr rangefinderData = 
  core::Channel<sawyer::DataRangefinders>::requestData(polling_timeout_);
  publishRawData();
  if(rangefinderData){
#ifdef DEBUG_INFO
  ROS_INFO("Received rangefinder Data, counts:%d, data[0]:%.2lf , data[1]:%.2lf, data[2]:%.2lf , data[3]:%.2lf, data[4]:%.2lf.",
	   (int)rangefinderData->getRangefinderCount(),rangefinderData->getDistance(0),rangefinderData->getDistance(1),rangefinderData->getDistance(2),rangefinderData->getDistance(3),rangefinderData->getDistance(4));
#endif
    
  }
  
  core::Channel<sawyer::DataRangefinderTimings>::Ptr rangefinderDataAndTime = 
  core::Channel<sawyer::DataRangefinderTimings>::requestData(polling_timeout_);
  publishRawData();
  if(rangefinderDataAndTime){
#ifdef DEBUG_INFO 
    ROS_INFO("Received rangefinder Data and time, counts:%d, data[0]:%.2lf, time[0]:%d, data[1]:%.2lf, time[1]:%d, data[2]:%.2lf, time[2]:%d, data[3]:%.2lf, time[3]:%d, data[4]:%.2lf, time[4]:%d.",
	   (int)rangefinderDataAndTime->getRangefinderCount(),rangefinderDataAndTime->getDistance(0),rangefinderDataAndTime->getAcquisitionTime(0),
	   rangefinderDataAndTime->getDistance(1),rangefinderDataAndTime->getAcquisitionTime(1),
	   rangefinderDataAndTime->getDistance(2),rangefinderDataAndTime->getAcquisitionTime(2),
	   rangefinderDataAndTime->getDistance(3),rangefinderDataAndTime->getAcquisitionTime(3),
	   rangefinderDataAndTime->getDistance(4),rangefinderDataAndTime->getAcquisitionTime(4));
#endif 
    }

}
void rochHardware::getPlatAccData()
{

  core::Channel<sawyer::DataPlatformAcceleration>::Ptr getPlatformAccData = 
  core::Channel<sawyer::DataPlatformAcceleration>::requestData(polling_timeout_);
  publishRawData();
  if(getPlatformAccData){
#ifdef DEBUG_INFO 
        ROS_INFO("Received acc_x:%.2lf, acc_y:%.2lf, acc_z:%.2lf.",getPlatformAccData->getX(),getPlatformAccData->getY(),getPlatformAccData->getZ());
#endif 
    sixGyro.acc.X = getPlatformAccData->getX() - sixGyro.acc.X_Offset;
    sixGyro.acc.Y = getPlatformAccData->getY() - sixGyro.acc.Y_Offset;
    sixGyro.acc.Z = getPlatformAccData->getZ() - sixGyro.acc.Z_Offset;
    
    linear_acceleration[0] = sixGyro.acc.Y;
    linear_acceleration[1] = sixGyro.acc.X;
    linear_acceleration[2] = sixGyro.acc.Z;
  }
  
}

void rochHardware::getPlatformName()
{ 
  core::Channel<sawyer::DataPlatformName>::Ptr getPlatformName = 
  core::Channel<sawyer::DataPlatformName>::requestData(polling_timeout_);
  publishRawData();
 #ifdef DEBUG_INFO 
  if(getPlatformName){
  std::cout<<"Received platform name:"<<getPlatformName->getName()<<std::endl;   
  }
#endif
}

void rochHardware::getDifferentControlConstantData()
{

  core::Channel<sawyer::DataDifferentialControl>::Ptr getDifferentControlConstantData = 
  core::Channel<sawyer::DataDifferentialControl>::requestData(polling_timeout_);
  publishRawData();
  if(getDifferentControlConstantData){
#ifdef DEBUG_INFO 
     ROS_INFO("Received Data of Differential Control Data, Left_P:%.2lf, Left_I:%.2lf, Left_D:%.2lf,"
	       "right_P:%.2lf, right_I:%.2lf, right_D:%.2lf.",
	   getDifferentControlConstantData->getLeftP(),getDifferentControlConstantData->getLeftI(),getDifferentControlConstantData->getLeftD(),
	   getDifferentControlConstantData->getRightP(),getDifferentControlConstantData->getRightI(),getDifferentControlConstantData->getRightD());
#endif 
   }
    
 }

  
  /**
  * Register diagnostic tasks with updater class
  */
  void rochHardware::initializeDiagnostics()
  {
    core::Channel<sawyer::DataPlatformInfo>::Ptr info =
      core::Channel<sawyer::DataPlatformInfo>::requestData(polling_timeout_);
      if(info){
      std::ostringstream hardware_id_stream;
      hardware_id_stream << "roch " << info->getModel() << "-" << info->getSerial();
      diagnostic_updater_.setHardwareID(hardware_id_stream.str());
      diagnostic_updater_.add(system_status_task_);
      diagnostic_updater_.add(power_status_task_);
      diagnostic_updater_.add(safety_status_task_);
      diagnostic_updater_.add(software_status_task_);
    }
    diagnostic_publisher_ = nh_.advertise<roch_msgs::RochStatus>("status", 10);
  
    raw_data_stream_publisher = nh_.advertise<sensor_msgs::Imu>("debug/raw_data_stream",100);
    imu_data_publisher_ = nh_.advertise<sensor_msgs::Imu>("sensor/imu_data",10);
    raw_data_command_publisher_ = nh_.advertise<std_msgs::String>("debug/raw_data_command",100);
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  * 注册模型关联
  */
  void rochHardware::registerControlInterfaces()
  {    
	 ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
	 ("front_right_wheel");//("rear_left_wheel")("rear_right_wheel");//原本 4驱
	imuMsgData.name="roch_sensor_controller/imu/data";
	imuMsgData.frame_id="imu_link";
	#if 1
	/**/
	orientation_covariance[0] =  1e9;//1e6;//DBL_MAX;
	orientation_covariance[1] =  0;
	orientation_covariance[2] =  0;
	orientation_covariance[3] =  0;
	orientation_covariance[4] =  1e9;//1e6;//DBL_MAX;
	orientation_covariance[5] =  0;
	orientation_covariance[6] =  0;
	orientation_covariance[7] =  0;
	orientation_covariance[8] =  0.05;//1e-6;//0.05;
	
	angular_velocity_covariance[0] =  1e9;//DBL_MAX;
	angular_velocity_covariance[1] =  0;
	angular_velocity_covariance[2] =  0;
	angular_velocity_covariance[3] =  0;
	angular_velocity_covariance[4] =  1e9;//DBL_MAX;
	angular_velocity_covariance[5] =  0;
	angular_velocity_covariance[6] =  0;
	angular_velocity_covariance[7] =  0;
	angular_velocity_covariance[8] =  0.05;//0.05;
	
	linear_acceleration_covariance[0] = 1e9;//-1;
	linear_acceleration_covariance[1] = 0;
	linear_acceleration_covariance[2] = 0;
	linear_acceleration_covariance[3] = 0;
	linear_acceleration_covariance[4] = 1e9;//0;
	linear_acceleration_covariance[5] = 0;
	linear_acceleration_covariance[6] = 0;
	linear_acceleration_covariance[8] = 0;
	linear_acceleration_covariance[8] = 1e-9;//0;
#endif
	geometry_msgs::Quaternion orien =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, sixGyro.angle);
	orientation[0] = orien.y;
	orientation[1] = orien.x;
	orientation[2] = sixGyro.angle;
	orientation[3] = orien.w;
	imuMsgData.orientation = orientation;
	imuMsgData.orientation_covariance =  orientation_covariance;
	imuMsgData.angular_velocity_covariance = angular_velocity_covariance;
	imuMsgData.linear_acceleration_covariance = linear_acceleration_covariance;
	angular_velocity[0] = 0.0;
	angular_velocity[1] = 0.0;
	angular_velocity[2] = sixGyro.angle_rate;
	imuMsgData.angular_velocity = angular_velocity;
	
	linear_acceleration[0] = sixGyro.acc.Y;
	linear_acceleration[1] = sixGyro.acc.X;
	linear_acceleration[2] = -sixGyro.acc.Z;
	
	imuMsgData.linear_acceleration = linear_acceleration;
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
#ifdef DEBUG_INFO 
          std::cout<<"Received joint_names["<<i<<"]:"<<joint_names[i]<<std::endl;
       ROS_INFO("Received joint[%d].position:%.2lf, joint[%d].velocity:%.2lf, joint[%d].effort:%.2lf",
						
						i,joints_[i].position,
						i, joints_[i].velocity,
						i,joints_[i].effort);

#endif 
   }
    hardware_interface::ImuSensorHandle imu_sensor_handle(imuMsgData);
   
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&imu_sensor_interface_);
  }

  /**
  * External hook to trigger diagnostic update
  * 外部触发诊断更新
  */
  void rochHardware::updateDiagnostics()
  {
    diagnostic_updater_.force_update();
    roch_status_msg_.header.stamp = ros::Time::now();
    diagnostic_publisher_.publish(roch_status_msg_);
  }
  
  
  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  * 从MCU中得到最新的速度和编码器，并保存到ros_control中的关节结构中
  */
  void rochHardware::updateJointsFromHardware()
  {
#ifdef DEBUG_INFO 
        for(int i=0;i<2;i++){ 
      ROS_INFO("Received joint[%d].position:%.2lf, joint[%d].velocity:%.2lf, joint[%d].effort:%.2lf",						
						i,joints_[i].position,
						i, joints_[i].velocity,
						i,joints_[i].effort);
      
    }
#endif 

    core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      polling_timeout_);
      publishRawData();
 
    if (enc)
    {
      ROS_DEBUG_STREAM("Received travel information (L:" << enc->getTravel(LEFT) << " R:" << enc->getTravel(RIGHT) << ")");
#ifdef DEBUG_INFO 
      ROS_INFO("Received travel information of encoders, Left:%.2lf, Right:%.2lf.",enc->getTravel(LEFT),enc->getTravel(RIGHT));
#endif
		for (int i = 0; i < 2; i++) //新本 双驱
      {
        double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;//linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;  //odom.position.x

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0)
        {
          joints_[i].position += delta;
#ifdef DEBUG_INFO 
	  ROS_INFO("jiounts_[%d].postion:%lf,delta:%lf.",i,joints_[i].position,delta);
#endif
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
#ifdef DEBUG_INFO 
	    ROS_INFO("jiounts_[%d].position_offset:%lf,delta:%lf.",i,joints_[i].position_offset,delta);
          ROS_DEBUG("Dropping overflow measurement from encoder");
#endif
        }
      }
    }
    core::Channel<sawyer::DataDifferentialSpeed>::Ptr speed = core::Channel<sawyer::DataDifferentialSpeed>::requestData(
      polling_timeout_);
    publishRawData();
    if (speed)
    {
      ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed() << " R:" << speed->getRightSpeed() << ")");
 #ifdef DEBUG_INFO      
      ROS_INFO("Received linear speed information, Left speed:%.2lf, Right speed:%.2lf.",speed->getLeftSpeed(),speed->getRightSpeed());
#endif
 		for (int i = 0; i < 2; i++)//新本 双驱
      {
        if (i % 2 == LEFT)//原本 4驱动
        {
          joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
#ifdef DEBUG_INFO 
	  ROS_INFO("jiounts_[%d].velocity:%lf.",i,joints_[i].velocity);
#endif
        }
        else
        { // assume RIGHT
	  
#ifdef DEBUG_INFO 
	  ROS_INFO("jiounts_[%d].velocity:%lf.",i,joints_[i].velocity);
#endif
          joints_[i].velocity = linearToAngular(speed->getRightSpeed());
        }
      }
    }
    core::Channel<sawyer::DataVelocity>::Ptr overallspeed = core::Channel<sawyer::DataVelocity>::requestData(
      polling_timeout_);
      publishRawData();
    if (overallspeed)
    {
      ROS_DEBUG_STREAM("Received  speed information (speed:" << overallspeed->getTranslational() << " Rotational:" << overallspeed->getRotational()<<" Acceleration:"<<overallspeed->getTransAccel() << ")");
#ifdef DEBUG_INFO 
      ROS_INFO("Received  speed information,  speed:%.2lf, Rotational:%.2lf, Acceleration:%.2lf",overallspeed->getTranslational(),overallspeed->getRotational(),overallspeed->getTransAccel());     
#endif      
    }
    
    core::Channel<sawyer::Data6AxisYaw>::Ptr imuRateData = core::Channel<sawyer::Data6AxisYaw>::requestData(
     polling_timeout_);
     publishRawData();
     if(imuRateData){
      ROS_DEBUG_STREAM("Received  imu rate data information (Angle:" << imuRateData->getAngle() << " Angle rate:" << imuRateData->getAngleRate() << ")");
#ifdef DEBUG_INFO 
      ROS_INFO("Received  imu rate data information, Angle:%.2lf, Angle rate:%.2lf",imuRateData->getAngle(),imuRateData->getAngleRate());     
#endif
      sixGyro.angle = imuRateData->getAngle() - sixGyro.angle_offset;
      sixGyro.angle_rate = imuRateData->getAngleRate();
      	
      geometry_msgs::Quaternion orien =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, sixGyro.angle);
	orientation[0] = orien.x;
	orientation[1] = orien.y;
	orientation[2] = sixGyro.angle;
	orientation[3] = orien.w;
	imuMsgData.orientation = orientation;
#ifdef DEBUG_INFO
      ROS_INFO("Received imu msg data, orientation.x:%lf, orientation.y:%lf, orientation.z:%lf, orientation.w:%lf",
		  imuMsgData.orientation[0],imuMsgData.orientation[1],imuMsgData.orientation[2],imuMsgData.orientation[3]);
#endif
      angular_velocity[2] = sixGyro.angle_rate;      
      if(imu_data_publisher_.getNumSubscribers()>0){
      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.frame_id = "imu_link";
      msg->header.stamp = ros::Time::now();

      msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, sixGyro.angle);

      // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
      // set yaw covariance as very low, to make it dominate over the odometry heading when combined
      // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
      //| DBL_MAX      0        0   | 					|  DBL_MAX	0	0   |
      //|    0       DBL_MAX    0   | =>  orientation_covariance	|    0	     DBL_MAX    0   | => angular_velocity_covariance
      //|    0         0       0.05 |					|    0          0      0.05 |	
      msg->orientation_covariance[0] = DBL_MAX;
      msg->orientation_covariance[4] = DBL_MAX;
      msg->orientation_covariance[8] = 0.05;

      // fill angular velocity; we ignore acceleration for now
      msg->angular_velocity.z = sixGyro.angle_rate;

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg->angular_velocity_covariance[0] = DBL_MAX;
      msg->angular_velocity_covariance[4] = DBL_MAX;
      msg->angular_velocity_covariance[8] = 0.05;

      imu_data_publisher_.publish(msg);      
      }
    }
    if ( ros::ok() && (raw_data_stream_publisher.getNumSubscribers() > 0) )
  {
    // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
   // gyroData.x  = 0;
   // gyroData.y  = 0;
   // gyroData.z  = overallspeed->getRotational();;

    ros::Time now = ros::Time::now();
    ros::Duration interval(0.01); // Time interval between each sensor reading.
  
      // Each sensor reading has id, that circulate 0 to 255.
      //msg->header.frame_id = std::string("imu_link_" + boost::lexical_cast<std::string>((unsigned int)data.frame_id+i));
      msg->header.frame_id = "imu_link";

      // Update rate of 3d gyro sensor is 100 Hz, but robot's update rate is 50 Hz.
      // So, here is some compensation.
      // See also https://github.com/yujinrobot/kobuki/issues/216
      msg->header.stamp = now;

      // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
      msg->angular_velocity.x = 0;
      msg->angular_velocity.y = 0;
      msg->angular_velocity.z = overallspeed->getRotational();

      raw_data_stream_publisher.publish(msg);
    
  }
 
    getRangefinderData();// get getDistance data;
    getDifferentControlConstantData();
    getPlatAccData();
    getPlatformName();
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  * 通过关节结构，在ros_control中得到最新的速度命令，然后发送到MCU中
  */
  void rochHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);

    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);
#ifdef DEBUG_INFO 
    ROS_INFO("diff_speed_left:%lf,joints_[LEFT].velocity_command:%lf.",diff_speed_left,joints_[LEFT].velocity_command);
    ROS_INFO("diff_speed_right:%lf,joints_[RIGHT].velocity_command:%lf.",diff_speed_left,joints_[RIGHT].velocity_command);
#endif
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    core::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
    publishRawData();
  }
   /**
  * Get latest velocity commands from ros_control via joint structure, and send overall speed to MCU
  * 通过关节结构，在ros_control中得到最新的速度命令，然后发送到MCU中
  */
  void rochHardware::writeOverallSpeedCommandsToHardware()
  {    
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    publishRawData();
    core::controloverallSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);   
    publishRawData();
  }
  
void rochHardware::publishRawData()
{
  if ( raw_data_command_publisher_.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
        std::ostringstream ostream;
      sawyer::base_data::RawData data =sawyer::Transport::instance().getdata();
      ostream << "{ " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < data.length; i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(data.data[i]) << " " << std::dec;
      ostream << "}";
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      if (ros::ok())
      {
        raw_data_command_publisher_.publish(msg);
      }
    }
}

  /**
  * Update diagnostics with control loop timing information
  */
  void rochHardware::reportLoopDuration(const ros::Duration &duration)
  {
    software_status_task_.updateControlFrequency(1 / duration.toSec());
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void rochHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * roch reports travel in metres, need radians for ros_control RobotHW
  */
  double rochHardware::linearToAngular(const double &travel) const
  {
//    ROS_INFO("linear To angular :%lf.",travel / wheel_diameter_ * 2);
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, roch needs m/s,
  */
  double rochHardware::angularToLinear(const double &angle) const
  {
//    ROS_INFO("angular To linear :%lf.",angle * wheel_diameter_ / 2);
    return angle * wheel_diameter_ / 2;
  }


}  // namespace roch_base
