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

#include "roch_base/core/sawyer.h"
#include <ros/ros.h>

namespace
{
  std::string port_;
}

namespace core
{

  void reconnect()
  {
    if (port_.empty())
    {
      throw std::logic_error("Can't reconnect when port is not configured");
    }
    ROS_INFO_STREAM("Connecting to roch on port " << port_ << "...");
    sawyer::Transport::instance().configure(port_.c_str(), 3);
  }

  void connect(std::string port)
  {
    port_ = port;
    reconnect();
  }

  void configureLimits(double max_speed, double max_accel)
  {

    bool success = false;
    while (!success)
    {
      try
      {
	
        sawyer::SetMaxAccel(max_accel, max_accel).send();
        sawyer::SetMaxSpeed(max_speed, max_speed).send();
        success = true;
      }
      catch (sawyer::Exception *ex)
      {
        ROS_ERROR_STREAM("Error configuring velocity and accel limits: " << ex->message);
        reconnect();
      }
    }
  }

  void controlSpeed(double speed_left, double speed_right, double accel_left, double accel_right)
  {
    bool success = false;
    while (!success)
    {
      try
      {
	sawyer::SetDifferentialSpeed(speed_left, speed_right, accel_left, accel_right).send();
        success = true;
      }
      catch (sawyer::Exception *ex)
      {
        ROS_ERROR_STREAM("Error sending speed and accel command: " << ex->message);
        reconnect();
      }
    }
  }
    void controloverallSpeed(double speed_left, double speed_right, double accel_left, double accel_right)
  {
    bool success = false;
    double lin_vel = (speed_left + speed_right)*0.5;
    double ang_vel = (speed_left - speed_right)/(2*0.12);
    ROS_INFO("speed_left:%.2lf, speed_right:%.2lf, ang_vel:%.2lf",speed_left,speed_right,ang_vel);
    
    double acc = (accel_left + accel_right)*0.5;
    
    while (!success)
    {
      try
      {
	sawyer::SetVelocity(lin_vel,ang_vel,acc).send();
        success = true;
      }
      catch (sawyer::Exception *ex)
      {
        ROS_ERROR_STREAM("Error sending velocity setpt command: " << ex->message);
        reconnect();
      }
    }
  }

}
