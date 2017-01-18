/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /roch_safety_controller/include/roch_safety_controller/safety_controller.hpp
 *
 * @brief roch-specific safety controller
 *
 * This controller uses roch's bumper, cliff and wheel drop sensors to ensure safe operation.
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <yocs_controllers/default_controller.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <roch_msgs/CliffEvent.h>
#include <roch_msgs/UltEvent.h>
#include <roch_msgs/PSDEvent.h>

namespace sawyer
{

/**
 * @ brief Keeps track of safety-related events and commands roch to move accordingly
 *
 * The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two,
 * roch is commanded to move back. In the latter case, roch is stopped. All commands stop when the
 * event condition disappears. In the case of lateral bump/cliff, robot also spins a bit, what makes
 * easier to escape from the risk.
 *
 * This controller can be enabled/disabled.
 * The safety states (bumper pressed etc.) can be reset. WARNING: Dangerous!
 */
class SafetyController : public yocs::Controller
{
public:
  SafetyController(ros::NodeHandle& nh, std::string& name) :
    Controller(),
    nh_(nh),
    name_(name),
    cliff_left_detected_(false),
    cliff_right_detected_(false), 
    ult_left_detected_(false),
    ult_center_detected_(false),
    ult_right_detected_(false),
    psd_left_detected_(false),
    psd_center_detected_(false),
    psd_right_detected_(false),
    last_event_time_(ros::Time(0)),
    msg_(new geometry_msgs::Twist()){};
  ~SafetyController(){};

  /**
   * Set-up necessary publishers/subscribers and variables
   * @return true, if successful
   */
  bool init()
  {
    //how long to keep sending messages after a bump, cliff, or wheel drop stops
    double time_to_extend_ult_cliff_psd_events;
    nh_.param("time_to_extend_ult_cliff_psd_events", time_to_extend_ult_cliff_psd_events, 0.0);
    time_to_extend_bump_cliff_events_ = ros::Duration(time_to_extend_ult_cliff_psd_events);
    enable_controller_subscriber_ = nh_.subscribe("enable", 10, &SafetyController::enableCB, this);
    disable_controller_subscriber_ = nh_.subscribe("disable", 10, &SafetyController::disableCB, this);
    cliff_event_subscriber_  = nh_.subscribe("events/cliff",  10, &SafetyController::cliffEventCB, this);    
    ult_event_subscriber_ = nh_.subscribe("events/ult", 10, &SafetyController::ultEventCB, this);
    psd_event_subscriber_ = nh_.subscribe("events/psd", 10, &SafetyController::psdEventCB, this);
    reset_safety_states_subscriber_ = nh_.subscribe("reset", 10, &SafetyController::resetSafetyStatesCB, this);
    velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
    return true;
  };

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spin();

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
  ros::Subscriber cliff_event_subscriber_, ult_event_subscriber_, psd_event_subscriber_;
  ros::Subscriber reset_safety_states_subscriber_;
  ros::Publisher controller_state_publisher_, velocity_command_publisher_;
  bool cliff_left_detected_, cliff_right_detected_;
  bool ult_left_detected_, ult_center_detected_, ult_right_detected_;
  bool psd_left_detected_, psd_center_detected_, psd_right_detected_;
  ros::Duration time_to_extend_bump_cliff_events_;
  ros::Time last_event_time_;

  geometry_msgs::TwistPtr msg_; // velocity command

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::EmptyConstPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::EmptyConstPtr msg);


  /**
   * @brief Keeps track of cliff detection
   * @param msg incoming topic message
   */
  void cliffEventCB(const roch_msgs::CliffEventConstPtr msg);
  
  
  /**
   * @brief Keeps track of ult detection
   * @param msg incoming topic message
   */
  void ultEventCB(const roch_msgs::UltEventConstPtr msg);
  
  
  /**
   * @brief Keeps track of PSD detection
   * @param msg incoming topic message
   */
  void psdEventCB(const roch_msgs::PSDEventConstPtr msg);


  /**
   * @brief Callback for resetting safety variables
   *
   * Allows resetting bumper, cliff and wheel drop states.
   * DANGEROUS!
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg);
};


void SafetyController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void SafetyController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

void SafetyController::cliffEventCB(const roch_msgs::CliffEventConstPtr msg)
{
  if ((msg->leftState == roch_msgs::CliffEvent::CLIFF) || (msg->rightState == roch_msgs::CliffEvent::CLIFF) )
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Cliff detected. Moving backwards. [" << name_ << "]");
    if((msg->leftState == roch_msgs::CliffEvent::CLIFF))
      cliff_left_detected_   = true; 
    if((msg->rightState == roch_msgs::CliffEvent::CLIFF))
      cliff_right_detected_   = true; 
  }
  else // roch_msgs::CliffEvent::FLOOR
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");
    if((msg->leftState == roch_msgs::CliffEvent::FLOOR))
      cliff_left_detected_   = false; 
    if((msg->rightState == roch_msgs::CliffEvent::FLOOR))
      cliff_right_detected_   = false; 
  }
};

void SafetyController::ultEventCB(const roch_msgs::UltEventConstPtr msg)
{
  if ((msg->leftState == roch_msgs::UltEvent::NEAR) || (msg->rightState == roch_msgs::UltEvent::NEAR) || (msg->centerState == roch_msgs::UltEvent::NEAR) )
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("Ult detected. Moving backwards. [" << name_ << "]");
    if((msg->leftState == roch_msgs::UltEvent::NEAR))
      ult_left_detected_ = true;
    if((msg->centerState == roch_msgs::UltEvent::NEAR))
      ult_center_detected_ = true;
    if((msg->rightState == roch_msgs::UltEvent::NEAR))
      ult_right_detected_ = true;
  }
  else // roch_msgs::UltEvent::NORMAL
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");
    if((msg->leftState == roch_msgs::UltEvent::NORMAL))
      ult_left_detected_ = false;
    if((msg->centerState == roch_msgs::UltEvent::NORMAL))
      ult_center_detected_ = false;
    if((msg->rightState == roch_msgs::UltEvent::NORMAL))
      ult_right_detected_ = false;
  }
};


void SafetyController::psdEventCB(const roch_msgs::PSDEventConstPtr msg)
{
  if ((msg->leftState == roch_msgs::PSDEvent::NEAR) || (msg->rightState == roch_msgs::PSDEvent::NEAR) || (msg->centerState == roch_msgs::PSDEvent::NEAR) )
  {
    last_event_time_ = ros::Time::now();
    ROS_DEBUG_STREAM("PSD detected. Moving backwards. [" << name_ << "]");
    if((msg->leftState == roch_msgs::PSDEvent::NEAR))
      psd_left_detected_ = true;
    if((msg->centerState == roch_msgs::PSDEvent::NEAR))
      psd_center_detected_ = true;
    if((msg->rightState == roch_msgs::PSDEvent::NEAR))
      psd_right_detected_ = true;
  }
  else // roch_msgs::PSDEvent::NORMAL
  {
    ROS_DEBUG_STREAM("Not detecting any cliffs. Resuming normal operation. [" << name_ << "]");
    if((msg->leftState == roch_msgs::PSDEvent::NORMAL))
      psd_left_detected_ = false;
    if((msg->centerState == roch_msgs::PSDEvent::NORMAL))
      psd_center_detected_ = false;
    if((msg->rightState == roch_msgs::PSDEvent::NORMAL))
      psd_right_detected_ = false;
  }
};


void SafetyController::resetSafetyStatesCB(const std_msgs::EmptyConstPtr msg)
{
  cliff_left_detected_   = false;
  cliff_right_detected_  = false;
  ult_left_detected_	 = false;
  ult_center_detected_	 = false;
  ult_right_detected_	 = false;
  psd_left_detected_	 = false;
  psd_center_detected_	 = false;
  psd_right_detected_	 = false;
  ROS_WARN_STREAM("All safety states have been reset to false. [" << name_ << "]");
}

void SafetyController::spin()
{
  if (this->getState())
  { 
    if (  psd_center_detected_ || ult_center_detected_) //ult_center_detected_: disable due to the ult not stable
    {
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_.publish(msg_);
    }
    else if (cliff_left_detected_ || psd_left_detected_ || ult_left_detected_)//ult_left_detected_: ult_center_detected_ disable due to the ult not stable
    {
      // left bump/cliff; also spin a bit to the right to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = -0.4;
      velocity_command_publisher_.publish(msg_);
    }
    else if (cliff_right_detected_ || psd_right_detected_ || ult_right_detected_)//ult_right_detected_: ult_center_detected_ disable due to the ult not stable
    {
      // right bump/cliff; also spin a bit to the left to make escape easier
      msg_.reset(new geometry_msgs::Twist());
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.4;
      velocity_command_publisher_.publish(msg_);
    }
    //if we want to extend the safety state and we're within the time, just keep sending msg_
     if (time_to_extend_bump_cliff_events_ > ros::Duration(1e-10) && 
	     ros::Time::now() - last_event_time_ < time_to_extend_bump_cliff_events_) {
      velocity_command_publisher_.publish(msg_);
    }
  }
};

} // namespace sawyer

#endif /* SAFETY_CONTROLLER_HPP_ */
