/**
 * @file /src/roch_sensorpc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>   
#include <ros/console.h>

#include "roch_sensorpc/roch_sensorpc.hpp"

#include <ros/ros.h>

namespace roch_sensorpc
{

void SensorPcNodelet::coreSensorCB(const roch_msgs::SensorState::ConstPtr& msg)
{
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->leftcliff  && ! prev_leftcliff && ! msg->rightcliff  && ! prev_rightcliff && 
      ! msg->leftult    && ! prev_leftult && ! msg->centerult  && ! prev_centerult && ! msg->rightult  && ! prev_rightult &&
      ! msg->leftpsd    && ! prev_leftpsd && ! msg->centerpsd  && ! prev_centerpsd && ! msg->rightpsd  && ! prev_rightpsd)
    return;

   prev_leftcliff  = msg->leftcliff;
   prev_rightcliff  = msg->rightcliff;
   prev_leftult  = msg->leftult;
   prev_centerult  = msg->centerult;
   prev_rightult  = msg->rightult;
   prev_leftpsd  = msg->leftpsd;
   prev_centerpsd  = msg->centerpsd;
   prev_rightpsd  = msg->rightpsd;
   
 
  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
  if ((cliff_enable_ && (msg->leftcliff & roch_msgs::SensorState::CLIFF )) || 
      (ult_enable_ && (msg->leftult  & roch_msgs::SensorState::NEAR)) || 
      (psd_enable_ && (msg->leftpsd  & roch_msgs::SensorState::NEAR)))
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
    ROS_INFO_STREAM("Enter left interrupt: cliff_enable:" << cliff_enable_ << "  ult_enable:"<<ult_enable_<<"  psd_enable:"<<psd_enable_<<".");
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if ((ult_enable_ && (msg->centerult & roch_msgs::SensorState::NEAR )) ||  
      (psd_enable_ && (msg->centerpsd & roch_msgs::SensorState::NEAR)))
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
    
    ROS_INFO_STREAM("Enter center interrupt: ult_enable:"<<ult_enable_<<" psd_enable:"<<psd_enable_<<".");
   }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if ((cliff_enable_ && (msg->rightcliff  & roch_msgs::SensorState::CLIFF)) ||
      (ult_enable_ && (msg->rightult & roch_msgs::SensorState::NEAR )) ||       
      (psd_enable_ && (msg->rightpsd  & roch_msgs::SensorState::NEAR)))
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
    ROS_INFO_STREAM("Enter right interrupt: cliff_enable:" << cliff_enable_ << "  ult_enable:"<<ult_enable_<<"  psd_enable:"<<psd_enable_<<".");
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  pointcloud_.header.stamp = msg->header.stamp;
  pointcloud_pub_.publish(pointcloud_);
}

void SensorPcNodelet::onInit()
{

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  std::string base_link_frame;
  double r, h, angle;
  bool ult_en,psd_en,cliff_en;
  private_nh_.param("pointcloud_radius", r, 0.25); pc_radius_ = r;
  private_nh_.param("pointcloud_height", h, 0.04); pc_height_ = h;
  private_nh_.param("side_point_angle", angle, 0.34906585); 
  private_nh_.param<std::string>("base_link_frame", base_link_frame, "/base_link");
  private_nh_.param<bool>("ult_enable", ult_enable_, true); 
  private_nh_.param<bool>("psd_enable", psd_enable_, true);  
  private_nh_.param<bool>("cliff_enable", cliff_enable_, true); 

  // Lateral points x/y coordinates; we need to store float values to memcopy later
  p_side_x_ = + pc_radius_*sin(angle); // angle degrees from vertical
  p_side_y_ = + pc_radius_*cos(angle); // angle degrees from vertical
  n_side_y_ = - pc_radius_*cos(angle); // angle degrees from vertical

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

  pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("pointcloud", 10);
  core_sensor_sub_ = nh.subscribe("core_sensors", 10, &SensorPcNodelet::coreSensorCB, this);
  ROS_INFO("init params: cliff_enable:%d  ult_enable:%d  psd_enable:%d.",cliff_enable_, ult_enable_, psd_enable_);

  ROS_INFO("Ult/psd/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
}

} // namespace roch_sensorpc

int main(int argc, char** argv){
  ros::init(argc,argv,"roch_sensorpc_nodelet");
  ros::NodeHandle nh, private_nh("~");
  roch_sensorpc::SensorPcNodelet sensorpc(nh,private_nh);
   
  ros::spin();
  
  return 0;
}
