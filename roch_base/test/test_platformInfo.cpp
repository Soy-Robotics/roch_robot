#include <ros/ros.h>
#include "sawyer_base/sawyer_hardware.h"

#include <boost/assign/list_of.hpp>
#include <float.h>
#define polling_timeout_ 20
using namespace std;
void publishRawData()
{
 
        std::ostringstream ostream;
      sawyer::base_data::RawData data =sawyer::Transport::instance().getdata();
      ostream << "command: { " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < data.length; i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(data.data[i]) << " " << std::dec;
      ostream << "}";
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      ROS_INFO("reveive command:%s",msg->data.c_str());
}

void getPlatformName()
{ 
  core::Channel<sawyer::DataPlatformName>::Ptr getPlatformName = 
  core::Channel<sawyer::DataPlatformName>::requestData(polling_timeout_);
  publishRawData();
  if(getPlatformName){
  std::cout<<"Received platform name:"<<getPlatformName->getName()<<std::endl;
    
  }
}

void getDifferentControlConstantData()
{

  core::Channel<sawyer::DataDifferentialControl>::Ptr getDifferentControlConstantData = 
  core::Channel<sawyer::DataDifferentialControl>::requestData(polling_timeout_);
  publishRawData();
  if(getDifferentControlConstantData){ 
     ROS_INFO("Received Data of Differential Control Data, Left_P:%.2lf, Left_I:%.2lf, Left_D:%.2lf,"
	       "right_P:%.2lf, right_I:%.2lf, right_D:%.2lf.",
	   getDifferentControlConstantData->getLeftP(),getDifferentControlConstantData->getLeftI(),getDifferentControlConstantData->getLeftD(),
	   getDifferentControlConstantData->getRightP(),getDifferentControlConstantData->getRightI(),getDifferentControlConstantData->getRightD());
   }
    
 }
namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};
int main(int argc, char** argv){
  
  std::string port = "/dev/ttyUSB0";
  core::connect(port);
  while(1){
    ROS_INFO("begin send requestData");
    core::Channel<sawyer::DataEncoders>::Ptr enc = core::Channel<sawyer::DataEncoders>::requestData(
      20);
    publishRawData();
      
    if (enc)
    {
		for (int i = 0; i < 2; i++)//新本 双驱
      {
        //joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
	ROS_INFO("joints_[%d].distance.x:%.2lf  ",i,enc->getTravel(i % 2));
      }
    }
    else
    {
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
  }
  return 0;
}
