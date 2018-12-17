#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

double m_vel_x, m_vel_y, m_vel_th;
geometry_msgs::TransformStamped odom_tf;
nav_msgs::Odometry odom_new;

void callback(const nav_msgs::Odometry::ConstPtr& odom_old)
{
	m_vel_x = odom_old->twist.twist.linear.x;
	m_vel_y = odom_old->twist.twist.linear.y;
	m_vel_th = odom_old->twist.twist.angular.z;
	//odom_new.pose.pose.orientation = odom_old->pose.pose.orientation;
	//odom_tf.transform.rotation = odom_old->pose.pose.orientation;
	//odom_new.header = odom_old->header;
  //odom_new.pose   = odom_old->pose;
	//odom_new.twist  = odom_old->twist;
	//odom_new.child_frame_id = "base_footprint";
	//odom_new.pose.pose.position.x = odom_new.pose.pose.position.x * 1.307;
	//odom_new.pose.pose.position.y = odom_new.pose.pose.position.y * 1.0;
	//pub.publish(odom_new);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_fix");
	ros::NodeHandle n;

	ros::Time m_currentTime;
	ros::Time m_lastTime;
	double m_pos_x, m_pos_y, m_pos_th;
	
	tf::TransformBroadcaster odom_bro;
	
	//geometry_msgs::Vector3 rpy;

	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Subscriber sub = n.subscribe("odom_old",10,callback);
	ros::Rate loop_rate(5);
	
	while(ros::ok())
	{
		m_currentTime = ros::Time::now();
		double dt = (m_currentTime - m_lastTime).toSec();//查询的时间间隔
		double delta_x = (m_vel_x * cos(m_pos_th) - m_vel_y * sin(m_pos_th)) * dt;//x方向增量
		double delta_y = (m_vel_x * sin(m_pos_th) + m_vel_y * cos(m_pos_th)) * dt;//y方向增量
		double delta_th = m_vel_th * dt;//角度增量
		m_pos_x += delta_x;
		m_pos_y += delta_y;
		m_pos_th += delta_th;
		//double roll, pitch, yaw;
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,m_pos_th*1.01);//四元数表达姿态
		odom_new.header.stamp = m_currentTime;
		odom_new.header.frame_id = "odom";
		odom_new.child_frame_id = "base_footprint";
		odom_new.pose.pose.position.x = m_pos_x*1.304;
		odom_new.pose.pose.position.y = m_pos_y;
		odom_new.pose.pose.position.z = 0.0;
		odom_new.twist.twist.linear.x = m_vel_x;
		odom_new.twist.twist.linear.y = m_vel_y;
		odom_new.twist.twist.linear.z = 0.0;
		odom_new.twist.twist.angular.x = 0.0;
		odom_new.twist.twist.angular.y = 0.0;
		odom_new.twist.twist.angular.z = m_vel_th;
	  odom_new.pose.pose.orientation = odom_quat;
	
		pub.publish(odom_new);

		odom_tf.header.stamp = m_currentTime;
		odom_tf.header.frame_id = "odom";
		odom_tf.child_frame_id = "base_footprint";
		odom_tf.transform.translation.x = m_pos_x*1.304;
		odom_tf.transform.translation.y = m_pos_y;
		odom_tf.transform.translation.z = 0.0;
		odom_tf.transform.rotation = odom_quat;
		odom_bro.sendTransform(odom_tf);
		m_vel_x = 0;
		m_vel_y = 0;
		m_vel_th = 0;
		loop_rate.sleep();
		m_lastTime = m_currentTime;
		ros::spinOnce();
		
	}
	
}
