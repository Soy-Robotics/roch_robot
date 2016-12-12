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
 * @file /roch_safety_controller/src/nodelet.cpp
 *
 * @brief Implementation for roch's safety controller nodelet
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/threads/thread.hpp>
#include "roch_safety_controller/safety_controller.hpp"
#include <boost/concept_check.hpp>


namespace sawyer
{

class SafetyControllerNodelet 
{
public:
  SafetyControllerNodelet() : shutdown_requested_(false) { onInit();};
  ~SafetyControllerNodelet()
  {
    ROS_DEBUG("Waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }
  void onInit()
  {
 
    // resolve node(let) name
    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos + 1);
    ROS_DEBUG("Initialising nodelet... [  %s  ]",name.c_str());
    controller_.reset(new SafetyController(nh, name));
    if (controller_->init())
    {
      ROS_DEBUG("roch initialised. Spinning up update thread ... [ %s ]",name.c_str());
      update_thread_.start(&SafetyControllerNodelet::update, *this);
      ROS_DEBUG("Nodelet initialised. [ %s ]",name.c_str());
    }
    else
    {
      ROS_DEBUG("Couldn't initialise nodelet! Please restart. [ %s ]",name.c_str());
    }
   
  }
private:
  void update()
  {
    ros::Rate spin_rate(10);
    controller_->enable(); // enable the controller when loading the nodelet
    while (! shutdown_requested_ && ros::ok())
    {
      controller_->spin();
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<SafetyController> controller_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
  ros::NodeHandle nh;
};

} // namespace roch

int main(int argc, char** argv){
  
  ros::init(argc,argv,"roch_safety_controller_nodelet");
  
  sawyer::SafetyControllerNodelet saftycon;
  
  ros::spin();
  
  return 0;
}