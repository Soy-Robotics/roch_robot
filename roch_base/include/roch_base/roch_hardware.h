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

#ifndef roch_BASE_roch_HARDWARE_H
#define roch_BASE_roch_HARDWARE_H

#include "roch_base/roch_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "roch_msgs/RochStatus.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <string>

#include <angles/angles.h>
#include <std_msgs/String.h>
#include "roch_base/core/Message.h"
#include "roch_base/core/serial.h"
namespace roch_base
{

  /**
  * Class representing roch hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class rochHardware :
    public hardware_interface::RobotHW
  {
    
  public:
    rochHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

    void updateDiagnostics();

    void reportLoopDuration(const ros::Duration &duration);
    
    void writeOverallSpeedCommandsToHardware();
    
    void getRangefinderData();
    
    void getPlatformName();
    
    void getPlatAccData();
    
    void getDifferentControlConstantData();
    
    void showRawData();
    
    void publishRawData();

  public:
    
    hardware_interface::ImuSensorHandle::Data imuMsgData;
    double orientation[4];
    double orientation_covariance [9];
    double angular_velocity_covariance[9] ;
    double linear_acceleration_covariance[9] ;
    double angular_velocity[3]   ;
    double linear_acceleration[3];
  private:

    void initializeDiagnostics();

    void resetTravelOffset();

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    // Diagnostics
    ros::Publisher diagnostic_publisher_;
    ros::Publisher imu_data_publisher_;
    ros::Publisher raw_data_stream_publisher;
    ros::Publisher raw_data_command_publisher_;
    roch_msgs::RochStatus roch_status_msg_;
    diagnostic_updater::Updater diagnostic_updater_;
    rochHardwareDiagnosticTask<sawyer::DataSystemStatus> system_status_task_;
    rochHardwareDiagnosticTask<sawyer::DataPowerSystem> power_status_task_;
    rochHardwareDiagnosticTask<sawyer::DataSafetySystemStatus> safety_status_task_;
    rochSoftwareDiagnosticTask software_status_task_;
  
      // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[2];//�±�  ˫��
    
    struct ThreeAxisGyro{
      double X;
      double X_Offset;
      double Y;
      double Y_Offset;
      double Z;
      double Z_Offset;
      
      ThreeAxisGyro() :
	X(0), X_Offset(0), Y(0), Y_Offset(0), Z(0), Z_Offset(0)
      { }
    }gyroData;
    
    struct SixAxisGyro{
      double angle;
      double angle_rate;   // rad/s
      double angle_rate_offset;
      double angle_offset;
      ThreeAxisGyro acc;
     
      SixAxisGyro() :
	angle(0), angle_rate(0), angle_rate_offset(0), angle_offset(0)
      { }
    }sixGyro;
#if 0
    struct ImuData{
      std::string name;                       ///< The name of the sensor
      std::string frame_id;                   ///< The reference frame to which this sensor is associated
      double* orientation;                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
      double* orientation_covariance;         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
      double* angular_velocity;               ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
      double* angular_velocity_covariance;    ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
      double* linear_acceleration;            ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
      double* linear_acceleration_covariance; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
 
      ImuData()
	  : name(),
	    frame_id(),
	    orientation(0),
	    orientation_covariance(0),
	    angular_velocity(0),
	    angular_velocity_covariance(0),
	    linear_acceleration(0),
	    linear_acceleration_covariance(0) {}
    }imuMsgData;
#endif
    
  };

}  // namespace roch_base
#endif  // roch_BASE_roch_HARDWARE_H
