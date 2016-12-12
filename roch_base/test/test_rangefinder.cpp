#include <ros/ros.h>
#include "roch_base/roch_hardware.h"

#include <boost/assign/list_of.hpp>
#include <float.h>
#define polling_timeout_ 20
using namespace std;
void publishRawData()
{
 #if 0
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
#endif
}

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};
int main(int argc, char** argv){
  
  std::string port = "/dev/ttyUSB0";
  core::connect(port);
  while(1){
  //  ROS_INFO("begin send requestData");
  core::Channel<sawyer::DataRangefinders>::Ptr rangefinderData = 
  core::Channel<sawyer::DataRangefinders>::requestData(polling_timeout_);
  publishRawData();
  if(rangefinderData){
  ROS_INFO("Received rangefinder Data, counts:%d, data[0]:%.2lf , data[1]:%.2lf, data[2]:%.2lf , data[3]:%.2lf, data[4]:%.2lf, data[5]:%.2lf, data[6]:%.2lf, data[7]:%.2lf.",
	   (int)rangefinderData->getRangefinderCount(),rangefinderData->getDistance(0),
							rangefinderData->getDistance(1),
							rangefinderData->getDistance(2),
							rangefinderData->getDistance(3),
							rangefinderData->getDistance(4),
							rangefinderData->getDistance(5),
							rangefinderData->getDistance(6),
							rangefinderData->getDistance(7));
  }
  #if 0
  core::Channel<sawyer::DataRangefinderTimings>::Ptr rangefinderDataAndTime = 
  core::Channel<sawyer::DataRangefinderTimings>::requestData(polling_timeout_);
  publishRawData();

  if(rangefinderDataAndTime){
    ROS_INFO("Received rangefinder Data and time, counts:%d, data[0]:%.2lf, time[0]:%d, "
							     " data[1]:%.2lf, time[1]:%d, "
							     " data[2]:%.2lf, time[2]:%d, "
							     " data[3]:%.2lf, time[3]:%d, "
							     " data[4]:%.2lf, time[4]:%d, "
							     " data[5]:%.2lf, time[5]:%d, "
							     " data[6]:%.2lf, time[6]:%d, "
							     " data[7]:%.2lf, time[7]:%d. ",
	   (int)rangefinderDataAndTime->getRangefinderCount(),rangefinderDataAndTime->getDistance(0),rangefinderDataAndTime->getAcquisitionTime(0),
	   rangefinderDataAndTime->getDistance(1),rangefinderDataAndTime->getAcquisitionTime(1),
	   rangefinderDataAndTime->getDistance(2),rangefinderDataAndTime->getAcquisitionTime(2),
	   rangefinderDataAndTime->getDistance(3),rangefinderDataAndTime->getAcquisitionTime(3),
	   rangefinderDataAndTime->getDistance(4),rangefinderDataAndTime->getAcquisitionTime(4),
	   rangefinderDataAndTime->getDistance(5),rangefinderDataAndTime->getAcquisitionTime(5),
	   rangefinderDataAndTime->getDistance(6),rangefinderDataAndTime->getAcquisitionTime(6),
	   rangefinderDataAndTime->getDistance(7),rangefinderDataAndTime->getAcquisitionTime(7));
    }
#endif
  }
  return 0;
}
