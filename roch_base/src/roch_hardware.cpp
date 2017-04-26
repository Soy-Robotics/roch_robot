/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
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
    private_nh_.param<double>("cliff_hegiht", cliff_height_, 0.1); //how tall can scan(meter)
    private_nh_.param<double>("ult_length", ult_length_, 0.1); //how far can scan(meter)
    private_nh_.param<double>("psd_length", PSD_length_, 0.1); //how far can scan(meter)
    private_nh_.param<std::string>("imu_link_frame", gyro_link_frame_, "imu_link");

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
  */
  void rochHardware::resetTravelOffset()
  {
    core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      polling_timeout_);
      publishRawData();
    if (enc)
    {
		for (int i = 0; i < 2; i++)
      {
        joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
        ROS_DEBUG_STREAM("joints_["<<i<<"].position_offset:"<<joints_[i].position_offset<<" .");
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
      ROS_DEBUG_STREAM("Received  imu rate data information, angle_offset:"<<imuRateData->getAngle()<<" .");        
      sixGyro.angle_offset = imuRateData->getAngle();
    }
    else{
       ROS_ERROR("Could not get imu data to calibrate rate offset");
    }
    core::Channel<sawyer::DataPlatformAcceleration>::Ptr getPlatformAccData = 
      core::Channel<sawyer::DataPlatformAcceleration>::requestData(polling_timeout_);
    publishRawData();
    if(getPlatformAccData){

      ROS_DEBUG_STREAM("Received acc_x:"<<getPlatformAccData->getX()<<", acc_y:"<<getPlatformAccData->getY()<<", acc_z:"<<getPlatformAccData->getZ()<<".");
 
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
      ROS_DEBUG_STREAM("Received rangefinder Data, counts:"<<(int)rangefinderData->getRangefinderCount()<<", data[0]:"<<rangefinderData->getDistance(0)<<" , data[1]:"<<rangefinderData->getDistance(1)<<", data[2]:"<<rangefinderData->getDistance(2)<<" , data[3]:"<<rangefinderData->getDistance(3)<<", data[4]:"<<rangefinderData->getDistance(4)<<".");

      publishCliffEvent(rangefinderData->getDistance(6),rangefinderData->getDistance(7));
      publishUltEvent(rangefinderData->getDistance(0),rangefinderData->getDistance(1),rangefinderData->getDistance(2));  
      publishPSDEvent(rangefinderData->getDistance(3),rangefinderData->getDistance(4),rangefinderData->getDistance(5));
    }
  
    core::Channel<sawyer::DataRangefinderTimings>::Ptr rangefinderDataAndTime = 
    core::Channel<sawyer::DataRangefinderTimings>::requestData(polling_timeout_);
    publishRawData();
    if(rangefinderDataAndTime){
      ROS_DEBUG_STREAM("Received rangefinder Data and time, counts:"<<(int)rangefinderDataAndTime->getRangefinderCount()<<
                     ", data[0]:"<<rangefinderDataAndTime->getDistance(0)<<", time[0]:"<<rangefinderDataAndTime->getAcquisitionTime(0)<<
                     ", data[1]:"<<rangefinderDataAndTime->getDistance(0)<<", time[1]:"<<rangefinderDataAndTime->getAcquisitionTime(0)<<
                     ", data[2]:"<<rangefinderDataAndTime->getDistance(0)<<", time[2]:"<<rangefinderDataAndTime->getAcquisitionTime(0)<<
                     ", data[3]:"<<rangefinderDataAndTime->getDistance(0)<<", time[3]:"<<rangefinderDataAndTime->getAcquisitionTime(0)<<
                     ", data[4]:"<<rangefinderDataAndTime->getDistance(0)<<", time[4]:"<<rangefinderDataAndTime->getAcquisitionTime(0)<<".");
    }
  }

  void rochHardware::getPlatAccData()
  {

    core::Channel<sawyer::DataPlatformAcceleration>::Ptr getPlatformAccData = 
    core::Channel<sawyer::DataPlatformAcceleration>::requestData(polling_timeout_);
    publishRawData();
    if(getPlatformAccData){

      ROS_DEBUG_STREAM("Received acc_x:"<<getPlatformAccData->getX()<<", acc_y:"<<getPlatformAccData->getY()<<", acc_z:"<<getPlatformAccData->getZ()<<".");
 
      sixGyro.acc.X = getPlatformAccData->getX() - sixGyro.acc.X_Offset;
      sixGyro.acc.Y = getPlatformAccData->getY() - sixGyro.acc.Y_Offset;
      sixGyro.acc.Z = getPlatformAccData->getZ() - sixGyro.acc.Z_Offset;
    
      linear_acceleration[0] = sixGyro.acc.X;
      linear_acceleration[1] = sixGyro.acc.Y;
      linear_acceleration[2] = sixGyro.acc.Z;
      imuMsgData.linear_acceleration = linear_acceleration;
    }
  
  }

  void rochHardware::getPlatformName()
  { 
    core::Channel<sawyer::DataPlatformName>::Ptr getPlatformName = 
    core::Channel<sawyer::DataPlatformName>::requestData(polling_timeout_);
    publishRawData();
 
    if(getPlatformName){
      ROS_DEBUG_STREAM("Received platform name:"<<getPlatformName->getName());   
    }

  }

  void rochHardware::getDifferentControlConstantData()
  {

    core::Channel<sawyer::DataDifferentialControl>::Ptr getDifferentControlConstantData = 
    core::Channel<sawyer::DataDifferentialControl>::requestData(polling_timeout_);
    publishRawData();
    if(getDifferentControlConstantData){
       ROS_DEBUG_STREAM("Received Data of Differential Control Data, Left_P:"<<getDifferentControlConstantData->getLeftP()<<", Left_I:"<<getDifferentControlConstantData->getLeftI()<<", Left_D:"<<getDifferentControlConstantData->getLeftD()<<
       ",right_P:"<<getDifferentControlConstantData->getRightP()<<", right_I:"<<getDifferentControlConstantData->getRightI()<<", right_D:"<<getDifferentControlConstantData->getRightD()<<".");
 
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
    raw_data_command_publisher_ = nh_.advertise<std_msgs::String>("debug/raw_data_command",100);
    
    //Obstacle detection publish
    cliff_event_publisher_ = nh_.advertise < roch_msgs::CliffEvent > ("events/cliff", 100);
    psd_event_publisher_ = nh_.advertise < roch_msgs::PSDEvent > ("events/psd", 100);
    ult_event_publisher_ = nh_.advertise < roch_msgs::UltEvent > ("events/ult", 100);
    sensor_state_publisher_ = nh_.advertise < roch_msgs::SensorState > ("core_sensors", 100);
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void rochHardware::registerControlInterfaces()
  {    
	 ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
	 ("front_right_wheel");
	imuMsgData.name="roch_sensor_controller/imu/data";
	imuMsgData.frame_id=gyro_link_frame_;

	orientation_covariance[0] =  0.001*0.001;
	orientation_covariance[1] =  0;
	orientation_covariance[2] =  0;
	orientation_covariance[3] =  0;
	orientation_covariance[4] =  0.001*0.001;
	orientation_covariance[5] =  0;
	orientation_covariance[6] =  0;
	orientation_covariance[7] =  0;
	orientation_covariance[8] =  0.001*0.001;
	
	angular_velocity_covariance[0] =  0.001*0.001;
	angular_velocity_covariance[1] =  0;
	angular_velocity_covariance[2] =  0;
	angular_velocity_covariance[3] =  0;
	angular_velocity_covariance[4] =  0.001*0.001;
	angular_velocity_covariance[5] =  0;
	angular_velocity_covariance[6] =  0;
	angular_velocity_covariance[7] =  0;
	angular_velocity_covariance[8] =  0.001*0.001;
	
	linear_acceleration_covariance[0] = 0.001*0.001;
	linear_acceleration_covariance[1] = 0;
	linear_acceleration_covariance[2] = 0;
	linear_acceleration_covariance[3] = 0;
	linear_acceleration_covariance[4] = 0.001*0.001;
	linear_acceleration_covariance[5] = 0;
	linear_acceleration_covariance[6] = 0;
	linear_acceleration_covariance[7] = 0;
	linear_acceleration_covariance[8] = 0.001*0.001;

	geometry_msgs::Quaternion orien =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, sixGyro.angle);
	orientation[0] = orien.y;
	orientation[1] = orien.x;
	orientation[2] = orien.z;
	orientation[3] = orien.w;
	imuMsgData.orientation =  orientation;
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
      std::cout<<"Received joint_names["<<i<<"]:"<<joint_names[i]<<std::endl;
      ROS_DEBUG_STREAM("Received joint["<<i<<"].position:"<<joints_[i].position<<", joint["<<i<<"].velocity:"<<joints_[i].velocity<<", joint["<<i<<"].effort:"<<joints_[i].effort<<"."); 
    }
    hardware_interface::ImuSensorHandle imu_sensor_handle(imuMsgData);
   
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&imu_sensor_interface_);
  }

  /**
  * External hook to trigger diagnostic update
  */
  void rochHardware::updateDiagnostics()
  {
    diagnostic_updater_.force_update();
    roch_status_msg_.header.stamp = ros::Time::now();
    diagnostic_publisher_.publish(roch_status_msg_);
  }
  
  
  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void rochHardware::updateJointsFromHardware()
  {

    for(int i=0;i<2;i++){ 
       ROS_DEBUG_STREAM("Received joint["<<i<<"].position:"<<joints_[i].position<<", joint["<<i<<"].velocity:"<<joints_[i].velocity<<", joint["<<i<<"].effort:"<<joints_[i].effort<<"."); 
    } 

    core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      polling_timeout_);
      publishRawData();
 
    if (enc)
    { 
      ROS_DEBUG_STREAM("Received travel information (L:" << enc->getTravel(LEFT) << " R:" << enc->getTravel(RIGHT) << ")");
      for (int i = 0; i < 2; i++)
      {
        double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;
        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 2.0)
        {
          joints_[i].position += delta;
	      ROS_DEBUG_STREAM("jiounts_["<<i<<"].postion:"<<joints_[i].position<<",delta:"<<delta<<".");
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
	      ROS_DEBUG_STREAM("jiounts_["<<i<<"].position_offset:"<<joints_[i].position_offset<<",delta:"<<delta<<".");
        }
      }
      
    }
    core::Channel<sawyer::DataDifferentialSpeed>::Ptr speed = core::Channel<sawyer::DataDifferentialSpeed>::requestData(
      polling_timeout_);
    publishRawData();
    if (speed)
    {
      ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed() << " R:" << speed->getRightSpeed() << ")");
      for (int i = 0; i < 2; i++)
      {
        if (i % 2 == LEFT)
        {
          joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
	      ROS_DEBUG_STREAM("jiounts_["<<i<<"].velocity:"<<joints_[i].velocity<<".");
        }
        else
        { // assume RIGHT
	      ROS_DEBUG_STREAM("jiounts_["<<i<<"].velocity:"<<joints_[i].velocity<<".");
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
    }
    
    core::Channel<sawyer::Data6AxisYaw>::Ptr imuRateData = core::Channel<sawyer::Data6AxisYaw>::requestData(
     polling_timeout_);
    publishRawData();
    if(imuRateData){
      ROS_DEBUG_STREAM("Received  imu rate data information (Angle:" << imuRateData->getAngle() << " Angle rate:" << imuRateData->getAngleRate() << ")");
      sixGyro.angle = imuRateData->getAngle() - sixGyro.angle_offset;
      sixGyro.angle_rate = imuRateData->getAngleRate();
      	
      geometry_msgs::Quaternion orien =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, sixGyro.angle);
      orientation[0] = orien.x;
      orientation[1] = orien.y;
      orientation[2] = orien.z;
      orientation[3] = orien.w;
      imuMsgData.orientation = orientation;
      ROS_DEBUG_STREAM("Received imu msg data, orientation.x:"<<imuMsgData.orientation[0]<<", orientation.y:"<<imuMsgData.orientation[1]<<", orientation.z:"<<imuMsgData.orientation[2]<<", orientation.w:"<<imuMsgData.orientation[3]<<".");

      angular_velocity[0] = 0.0;
      angular_velocity[1] = 0.0;
      angular_velocity[2] = sixGyro.angle_rate;
      imuMsgData.angular_velocity = angular_velocity;
    } 
    getPlatAccData();
    getRangefinderData();
    getDifferentControlConstantData();
    getPlatformName();
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void rochHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    ROS_DEBUG_STREAM("diff_speed_left:"<<diff_speed_left<<",joints_[LEFT].velocity_command:"<<joints_[LEFT].velocity_command<<".");
    ROS_DEBUG_STREAM("diff_speed_right:"<<diff_speed_left<<",joints_[RIGHT].velocity_command:"<<joints_[RIGHT].velocity_command<<".");

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    core::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
    publishRawData();
    
    publishSensorState();
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send overall speed to MCU
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

  void rochHardware::publishCliffEvent(const double &left, const double &right)
  {
    leftcliffevent.sensor = CliffEvent::Left;    
    if(left>cliff_height_)
       leftcliffevent.state = CliffEvent::Cliff;
    else    
       leftcliffevent.state = CliffEvent::Floor;
    leftcliffevent.leftbottom = left;
  
    rightcliffevent.sensor = CliffEvent::Right;
    if(right>cliff_height_)
       rightcliffevent.state = CliffEvent::Cliff;
    else    
       rightcliffevent.state = CliffEvent::Floor;
    rightcliffevent.rightbottom = right;   
  
    if (cliff_event_publisher_.getNumSubscribers()>0)
    {
      roch_msgs::CliffEventPtr msg(new roch_msgs::CliffEvent);
      switch(leftcliffevent.state) {
        case(CliffEvent::Floor) : { msg->leftState = roch_msgs::CliffEvent::FLOOR; break; }
        case(CliffEvent::Cliff) : { msg->leftState = roch_msgs::CliffEvent::CLIFF; break; }
        default: break;
      }
      switch(rightcliffevent.state) {
        case(CliffEvent::Floor) : { msg->rightState = roch_msgs::CliffEvent::FLOOR; break; }
        case(CliffEvent::Cliff) : { msg->rightState = roch_msgs::CliffEvent::CLIFF; break; }
        default: break;
      }
      msg->leftSensor = roch_msgs::CliffEvent::LEFT;
      msg->rightSensor = roch_msgs::CliffEvent::RIGHT;
      msg->leftBottom = leftcliffevent.leftbottom;
      msg->rightBottom = rightcliffevent.rightbottom;
      cliff_event_publisher_.publish(msg);
    }
  }

  void rochHardware::publishUltEvent(const double &left, const double &center, const double &right){
  
    leftultevent.sensor = UltEvent::Left;    
    if(left<ult_length_)
       leftultevent.state = UltEvent::Near;
    else    
       leftultevent.state = UltEvent::Normal;
    leftultevent.leftbottom = left;

    centerultevent.sensor = UltEvent::Center;    
    if(center<ult_length_)
       centerultevent.state = UltEvent::Near;
    else    
       centerultevent.state = UltEvent::Normal;
    centerultevent.centerbottom = center;
  
    rightultevent.sensor = UltEvent::Right;
    if(right<ult_length_)
       rightultevent.state = UltEvent::Near;
    else    
       rightultevent.state = UltEvent::Normal;
    rightultevent.rightbottom = right;   
  
    if (ult_event_publisher_.getNumSubscribers()>0)
    {
      roch_msgs::UltEventPtr msg(new roch_msgs::UltEvent);
      switch(leftultevent.state) {
        case(UltEvent::Normal) : { msg->leftState = roch_msgs::UltEvent::NORMAL; break; }
        case(UltEvent::Near) : { msg->leftState = roch_msgs::UltEvent::NEAR; break; }
        default: break;
      }
      switch(centerultevent.state) {
        case(UltEvent::Normal) : { msg->centerState = roch_msgs::UltEvent::NORMAL; break; }
        case(UltEvent::Near) : { msg->centerState = roch_msgs::UltEvent::NEAR; break; }
        default: break;
      }
      switch(rightultevent.state) {
        case(UltEvent::Normal) : { msg->rightState = roch_msgs::UltEvent::NORMAL; break; }
        case(UltEvent::Near) : { msg->rightState = roch_msgs::UltEvent::NEAR; break; }
        default: break;
      }
      msg->leftSensor = roch_msgs::UltEvent::LEFT;
      msg->centerSensor = roch_msgs::UltEvent::CENTER;
      msg->rightSensor = roch_msgs::UltEvent::RIGHT;
      msg->leftBottom = leftultevent.leftbottom;
      msg->centerBottom = centerultevent.centerbottom;
      msg->rightBottom = rightultevent.rightbottom;
      ult_event_publisher_.publish(msg);
    }
  }


  /*****************************************************************************
  ** Publish Sensor Stream Workers
  *****************************************************************************/
  void rochHardware::publishSensorState()
  {
    if(sensor_state_publisher_.getNumSubscribers()>0){
      cliffbottom.clear();
      ultbottom.clear();
      psdbottom.clear();
      roch_msgs::SensorState statecore;
      statecore.header.stamp = ros::Time::now();
      statecore.leftcliff = leftcliffevent.state;
      statecore.rightcliff = rightcliffevent.state;
      statecore.leftult = leftultevent.state;
      statecore.centerult = centerultevent.state;
      statecore.rightult = rightultevent.state;
      statecore.leftpsd = leftpsdevent.state;
      statecore.centerpsd = centerpsdevent.state;
      statecore.rightpsd = rightpsdevent.state;

      cliffbottom.push_back(leftcliffevent.leftbottom);
      cliffbottom.push_back(rightcliffevent.rightbottom);
    
      ultbottom.push_back(leftultevent.leftbottom);
      ultbottom.push_back(centerultevent.centerbottom);
      ultbottom.push_back(rightultevent.rightbottom);
    
      psdbottom.push_back(leftpsdevent.leftbottom);
      psdbottom.push_back(centerpsdevent.centerbottom);
      psdbottom.push_back(rightpsdevent.rightbottom);
      statecore.cliffbottom = cliffbottom;
      statecore.ultbottom = ultbottom;
      statecore.psdbottom = psdbottom;
      sensor_state_publisher_.publish(statecore);  
    }
  }

  void rochHardware::publishPSDEvent(const double& left, const double& center, const double& right)
  {
  
    leftpsdevent.sensor = PSDEvent::Left;    
    if(left<PSD_length_)
       leftpsdevent.state = PSDEvent::Near;
    else    
       leftpsdevent.state = PSDEvent::Normal;
    leftpsdevent.leftbottom = left;

    centerpsdevent.sensor = PSDEvent::Center;    
    if(center<PSD_length_)
       centerpsdevent.state = PSDEvent::Near;
    else    
       centerpsdevent.state = PSDEvent::Normal;
    centerpsdevent.centerbottom = center;
  
    rightpsdevent.sensor = PSDEvent::Right;
    if(right<PSD_length_)
       rightpsdevent.state = PSDEvent::Near;
    else    
       rightpsdevent.state = PSDEvent::Normal;
    rightpsdevent.rightbottom = right;   
  
    if (psd_event_publisher_.getNumSubscribers()>0)
    {
      roch_msgs::PSDEventPtr msg(new roch_msgs::PSDEvent);
      switch(leftpsdevent.state) {
        case(PSDEvent::Normal) : { msg->leftState = roch_msgs::PSDEvent::NORMAL; break; }
        case(PSDEvent::Near) : { msg->leftState = roch_msgs::PSDEvent::NEAR; break; }
        default: break;
      }
      switch(centerpsdevent.state) {
        case(PSDEvent::Normal) : { msg->centerState = roch_msgs::PSDEvent::NORMAL; break; }
        case(PSDEvent::Near) : { msg->centerState = roch_msgs::PSDEvent::NEAR; break; }
        default: break;
      }
      switch(rightpsdevent.state) {
        case(PSDEvent::Normal) : { msg->rightState = roch_msgs::PSDEvent::NORMAL; break; }
        case(PSDEvent::Near) : { msg->rightState = roch_msgs::PSDEvent::NEAR; break; }
        default: break;
      }
      msg->leftSensor = roch_msgs::PSDEvent::LEFT;
      msg->centerSensor = roch_msgs::PSDEvent::CENTER;
      msg->rightSensor = roch_msgs::PSDEvent::RIGHT;
      msg->leftBottom = leftpsdevent.leftbottom;
      msg->centerBottom = centerpsdevent.centerbottom;
      msg->rightBottom = rightpsdevent.rightbottom;
      psd_event_publisher_.publish(msg);
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
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, roch needs m/s,
  */
  double rochHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace roch_base
