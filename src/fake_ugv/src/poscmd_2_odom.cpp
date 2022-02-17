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
ros::Timer simulate_timer;

ackermann_msgs::AckermannDriveStamped ackermann_cmd;
nav_msgs::Odometry last_odom;
double speed = 0.0;
double yaw = 0.0;

double p_init_x, p_init_y, p_init_z;
double time_resolution = 0.01;
double wheelbase = 2.5;
double max_steer = 0.7854;
double min_speed = -5.555;
double max_speed = 15.278;

bool rcv_cmd = false;

void initParams()
{
	p_init_x = 0.0;
	p_init_y = 0.0;
	p_init_z = 0.0;
}

void pubOdom();
void normyaw(double& y);

void rcvVelCmdCallBack(const ackermann_msgs::AckermannDriveStamped cmd)
{	
	               rcv_cmd 	  = true;
	ackermann_cmd    = cmd;
    ROS_INFO("in ODOM, the cmd is: a=%f, steer=%f", cmd.drive.acceleration, cmd.drive.steering_angle);
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

// void pubOdom()
// {	
// 	nav_msgs::Odometry new_odom;

// 	new_odom.header.stamp       = ros::Time::now();
// 	new_odom.header.frame_id    = "world";

// 	if(rcv_cmd)
// 	{
// 		double delta = ackermann_cmd.drive.steering_angle;
// 		if (delta >= max_steer)
// 		{
// 			delta = max_steer;
// 		}else if (delta<= - max_steer)
// 		{
// 			delta = -max_steer;
// 		}

// 		Eigen::Quaterniond    q(	last_odom.pose.pose.orientation.w,
// 									last_odom.pose.pose.orientation.x,
// 									last_odom.pose.pose.orientation.y,
// 									last_odom.pose.pose.orientation.z  );

// 		Eigen::Matrix3d       R(q);
// 		double last_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);    
// 		last_yaw = yaw;              
// 		double last_x   	= last_odom.pose.pose.position.x;                  
// 		double last_y  	    = last_odom.pose.pose.position.y;               

// 		new_odom.pose.pose.position.x  = last_x + speed * cos(last_yaw) * time_resolution ;
// 		new_odom.pose.pose.position.y  = last_y +speed * sin(last_yaw) * time_resolution ;
// 		new_odom.pose.pose.position.z  = 0;
// 		new_odom.twist.twist.linear.x  = speed * cos(last_yaw);
// 		new_odom.twist.twist.linear.y  = speed * sin(last_yaw);
// 		new_odom.twist.twist.linear.z  = 0;
// 							double omega  = speed * tan(delta) / wheelbase;
// 		new_odom.twist.twist.angular.z = omega;
// 		speed = speed + ackermann_cmd.drive.acceleration * time_resolution;
// 		if (speed >= max_speed)
// 		{
// 			speed = max_speed;
// 		}else if (speed<= min_speed)
// 		{
// 			speed = min_speed;
// 		}
		
// 		yaw = yaw + omega * time_resolution;

// 		//cout << "omega = "<< omega <<endl;
// 		Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
// 		Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
// 		Eigen::Vector3d zC(0, 0, 1);
// 		Eigen::Matrix3d R2;
// 		R2.col(0) = xC;
// 		R2.col(1) = yC;
// 		R2.col(2) = zC;
// 		Eigen::Quaterniond q2(R2);
// 		new_odom.pose.pose.orientation.w = q2.w();
// 		new_odom.pose.pose.orientation.x = q2.x();
// 		new_odom.pose.pose.orientation.y = q2.y();
// 		new_odom.pose.pose.orientation.z = q2.z();
// 	}
// 	else
// 	{
// 		new_odom = last_odom;
// 	}

// 	last_odom = new_odom;
//     odom_pub.publish(new_odom);
// }

void simCallback(const ros::TimerEvent &e)
{
	static int k=0;
	nav_msgs::Odometry new_odom;

	new_odom.header.stamp       = ros::Time::now();
	new_odom.header.frame_id    = "world";

	double delta = ackermann_cmd.drive.steering_angle;
	if (delta >= max_steer)
	{
		delta = max_steer;
	}else if (delta<= - max_steer)
	{
		delta = -max_steer;
	}

	Eigen::Quaterniond    q(last_odom.pose.pose.orientation.w,
								last_odom.pose.pose.orientation.x,
								last_odom.pose.pose.orientation.y,
								last_odom.pose.pose.orientation.z  );

	Eigen::Matrix3d       R(q);
	double last_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);    
	last_yaw = yaw;              
	double last_x   	= last_odom.pose.pose.position.x;                  
	double last_y  	    = last_odom.pose.pose.position.y;               

	new_odom.pose.pose.position.x  = last_x + speed * cos(last_yaw) * time_resolution ;
	new_odom.pose.pose.position.y  = last_y +speed * sin(last_yaw) * time_resolution ;
	new_odom.pose.pose.position.z  = 0;
	new_odom.twist.twist.linear.x  = speed * cos(last_yaw);
	new_odom.twist.twist.linear.y  = speed * sin(last_yaw);
	new_odom.twist.twist.linear.z  = 0;
						double omega  = speed * tan(delta) / wheelbase;
	new_odom.twist.twist.angular.z = omega;
	speed = speed + ackermann_cmd.drive.acceleration * time_resolution;
	if (speed >= max_speed)
	{
		speed = max_speed;
	}else if (speed<= min_speed)
	{
		speed = min_speed;
	}
	
	yaw = yaw + omega * time_resolution;

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

	last_odom = new_odom;
	k++;
	if (k==2)
	{
    	odom_pub.publish(new_odom);
		k=0;
	}
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "ugv_kinematic_model_node");
    ros::NodeHandle nh( "~" );

	initParams();
  	nh.getParam("simulator/wheelbase", wheelbase);
	nh.getParam("simulator/max_steer", max_steer);
	nh.getParam("simulator/max_speed", max_speed);
	nh.getParam("simulator/min_speed", min_speed);
	  
    wheel_velocity_cmdsub  = nh.subscribe( "command", 1000, rcvVelCmdCallBack );
    		     odom_pub  = nh.advertise<nav_msgs::Odometry>("odometry", 10);
	
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

	ackermann_cmd.drive.acceleration = 0.0;
	ackermann_cmd.drive.steering_angle = 0.0;

    simulate_timer = nh.createTimer(ros::Duration(time_resolution), simCallback);
	ros::spin();


    // ros::Rate rate(100);

    // while(ros::ok()) 
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}