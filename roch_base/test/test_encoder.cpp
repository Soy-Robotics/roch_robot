#include <ros/ros.h>
#include "roch_base/roch_hardware.h"

#include <boost/assign/list_of.hpp>
#include <float.h>
#define polling_timeout_ 20
using namespace std;
namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};
double speed_x;
double speed_z;
void publishRawData()
{
 
        std::ostringstream ostream;
      sawyer::base_data::RawData data =sawyer::Transport::instance().getdata();
      ostream << " { " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < data.length; i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(data.data[i]) << " " << std::dec;
      ostream << "}";
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      ROS_INFO("reveive command:%s",msg->data.c_str());
}

void controloverallSpeed(double lin_vel, double ang_vel, double accel_left, double accel_right)
  {
    bool success = false;
    
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
       
      }
    }
  }
void speedCallBack(const geometry_msgs::Twist::ConstPtr& speed){
  
    cout<<"setspeed "<<speed_x<<"  "<<speed_z<<endl;
  speed_x = speed->linear.x;
  speed_z = speed->angular.z;
  
  
    controloverallSpeed(speed_x, speed_z, 0, 0);   
      
    publishRawData();
#if 1
     core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      polling_timeout_);
    publishRawData();
      
    if (enc)
    {
	//	for (int i = 0; i < 2; i++)//新本 双驱
   //   {
	//ROS_INFO("encoder[%d].distance.x:%.2lf  ",i,enc->getTravel(i % 2));
      ROS_INFO("encoder[0].distance.x:%.2lf ,encoder[1].distance.x:%.2lf . ",enc->getTravel(LEFT),enc->getTravel(RIGHT));
  //    }
    }
    else
    {
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
#endif 
      core::Channel<sawyer::DataEncodersRaw>::Ptr encoderraw = core::Channel<sawyer::DataEncodersRaw>::requestData(
      polling_timeout_);
      if(encoderraw){
	ROS_INFO("encoder[0].clicks:%.2d ,encoder[1].clicks:%.2d . ",encoderraw->getTicks(LEFT),encoderraw->getTicks(RIGHT));
      }else{      
	ROS_ERROR("Could not get encoder raw data to calibrate travel offset");	
      }
}
ros::Subscriber speedSubscriber_;
int main(int argc, char** argv){
  ros::init(argc,argv,"test_encoder");
  std::string port = "/dev/ttyUSB0";
  core::connect(port);
  
  ros::NodeHandle nh;
  speedSubscriber_ = nh.subscribe("/roch_velocity_controller/cmd_vel",100000,&speedCallBack);

    ROS_INFO("begin send requestData");
 
   ros::spin();
  return 0;
}
