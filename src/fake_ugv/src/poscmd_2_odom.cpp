#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

using namespace std;

ros::Subscriber wheel_velocity_cmdsub;
ros::Publisher  odom_pub;

ackermann_msgs::AckermannDriveStamped ackermann_cmd;
nav_msgs::Odometry last_odom;

double p_init_x, p_init_y, p_init_z;
// double time_resolution = 0.01;
double time_resolution = 0.2;
double wheelbase = 0.0;

bool rcv_cmd = false;

void initParams()
{
	p_init_x = 0.0;
	p_init_y = 0.0;
	p_init_z = 0.0;
}

void rcvVelCmdCallBack(const ackermann_msgs::AckermannDriveStamped cmd)
{	
	               rcv_cmd 	  = true;
	ackermann_cmd    = cmd;
}

void normyaw(double& y)
{
  if (y>M_PI)
  {
    y-=2*M_PI;
  }
  else if (y<-M_PI)
  {
    y+=2*M_PI;
  }
}

void pubOdom()
{	
	nav_msgs::Odometry new_odom;

	new_odom.header.stamp       = ros::Time::now();
	new_odom.header.frame_id    = "world";

	if(rcv_cmd)
	{
		Eigen::Quaterniond    q(	last_odom.pose.pose.orientation.w,
									last_odom.pose.pose.orientation.x,
									last_odom.pose.pose.orientation.y,
									last_odom.pose.pose.orientation.z  );

		Eigen::Matrix3d       R(q);
		Eigen::Vector2d       lvel(last_odom.twist.twist.linear.x,last_odom.twist.twist.linear.y);
		double last_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);                  
		double last_x   	= last_odom.pose.pose.position.x;                  
		double last_y  	    = last_odom.pose.pose.position.y;               


		new_odom.pose.pose.position.x  = last_x + ackermann_cmd.drive.speed * cos(last_yaw) * time_resolution ;
		new_odom.pose.pose.position.y  = last_y +ackermann_cmd.drive.speed * sin(last_yaw) * time_resolution ;
		new_odom.pose.pose.position.z  = 0;
		new_odom.twist.twist.linear.x  = ackermann_cmd.drive.speed * cos(last_yaw);
		new_odom.twist.twist.linear.y  = ackermann_cmd.drive.speed * sin(last_yaw);
		new_odom.twist.twist.linear.z  = 0;
							double omega  = ackermann_cmd.drive.speed * tan(ackermann_cmd.drive.steering_angle) / wheelbase;
		new_odom.twist.twist.angular.z = omega;

		double yaw = last_yaw + omega * time_resolution;
		normyaw(yaw);

		//cout << "omega = "<< omega <<endl;
		Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
		Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
		Eigen::Vector3d zC(0, 0, 1);
		Eigen::Matrix3d R2;
		R2.col(0) = xC;
		R2.col(1) = yC;
		R2.col(2) = zC;
		Eigen::Quaterniond q2(R2);
		new_odom.pose.pose.orientation.w = q2.w();
		new_odom.pose.pose.orientation.x = q2.x();
		new_odom.pose.pose.orientation.y = q2.y();
		new_odom.pose.pose.orientation.z = q2.z();
	}
	else
	{
		new_odom = last_odom;
	}

	last_odom = new_odom;
    odom_pub.publish(new_odom);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "ugv_kinematic_model_node");
    ros::NodeHandle nh( "~" );

	initParams();
  	nh.getParam("simulator/wheelbase", wheelbase);
	  
    wheel_velocity_cmdsub  = nh.subscribe( "command", 1, rcvVelCmdCallBack );
    		     odom_pub  = nh.advertise<nav_msgs::Odometry>("odometry", 1);
				 
	last_odom.header.stamp    = ros::Time::now();
	last_odom.header.frame_id = "world";                      
	last_odom.pose.pose.position.x = p_init_x;
	last_odom.pose.pose.position.y = p_init_y;
	last_odom.pose.pose.position.z = p_init_z;

	last_odom.pose.pose.orientation.w = 1;
	last_odom.pose.pose.orientation.x = 0;
	last_odom.pose.pose.orientation.y = 0;
	last_odom.pose.pose.orientation.z = 0;

	last_odom.twist.twist.linear.x = 0.0;
	last_odom.twist.twist.linear.y = 0.0;
	last_odom.twist.twist.linear.z = 0.0;

	last_odom.twist.twist.angular.x = 0.0;
	last_odom.twist.twist.angular.y = 0.0;
	last_odom.twist.twist.angular.z = 0.0;

    ros::Rate rate(1.0 / time_resolution);
    // ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}