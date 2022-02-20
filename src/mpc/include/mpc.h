#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <string.h>
#include <algorithm>
 
#include <OsqpEigen/OsqpEigen.h>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "cubic_spline_planner.h"

using namespace std;
using namespace Eigen;

class MPCState
{
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double theta = 0;
};

class MPC
{
private:
    // parameters
    /// algorithm param
    double du_th = 0.1;
    double dt = 0.2;
    double wheel_base = 2.5;
    int T = 5;
    int max_iter = 3;
    double Q[4] = {1, 1, 0.5, 0.5};
    double R[2] = {0.01, 0.01};
    double Rd[2] = {0.01, 1.0};
    /// constraints
    double max_steer = M_PI / 4;
    double max_dsteer = M_PI / 6;
    double max_csteer = M_PI / 6 * 0.2;
    double max_speed = 55.0 / 3.6;
    double min_speed = -20.0 / 3.6;
    double max_cv = 0.2;
    double max_accel = 1.0; 

    // MPC dataset
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;
    MPCState xbar[50];
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;

    // control data
    bool has_odom;
    bool receive_traj_ = false;
    bool control_a = true;
    double traj_duration_;
    double t_track = 0.0;
    ros::Time start_time_;
    MPCState now_state;

    // ros interface
	ros::NodeHandle node_;
    ros::Timer cmd_timer_;
    ros::Publisher pos_cmd_pub_, vis_pub, fake_odom_pub;
    ros::Subscriber odom_sub_, trajectory_sub_;
    ackermann_msgs::AckermannDriveStamped cmd;
    void cmdCallback(const ros::TimerEvent &e);
    void rcvOdomCallBack(nav_msgs::OdometryPtr msg);
    // void rcvTrajCallBack(... msg);

    // for test tracking performance
    bool in_test;
    cubic_spline_planner csp;
    vector<Eigen::Vector3d> csp_path;

    // MPC function
    void getLinearModel(const MPCState& s, double delta);
    void stateTrans(MPCState& s, double a, double delta);
    void predictMotion(void);
    void solveMPCV(void);
    void solveMPCA(void);
    void getCmd(void);

    // utils
    void pub_fake_odom(void)
    {
        MPCState temp = now_state;
        stateTrans(temp, output(0, 0), output(1, 0));
        nav_msgs::Odometry new_odom;

        normlize_theta(temp.theta);
        new_odom.header.stamp       = ros::Time::now();
	    new_odom.header.frame_id    = "world";
        new_odom.pose.pose.position.x  = temp.x;
		new_odom.pose.pose.position.y  = temp.y;
		new_odom.pose.pose.position.z  = 0;
		new_odom.twist.twist.linear.x  = temp.v * cos(temp.theta);
		new_odom.twist.twist.linear.y  = temp.v * sin(temp.theta);
		new_odom.twist.twist.linear.z  = 0;
							double omega  = temp.v * tan(cmd.drive.steering_angle) / wheel_base;
		new_odom.twist.twist.angular.z = omega;

		//cout << "omega = "<< omega <<endl;
		Eigen::Vector3d xC(cos(temp.theta), sin(temp.theta), 0);
		Eigen::Vector3d yC(-sin(temp.theta), cos(temp.theta), 0);
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
        fake_odom_pub.publish(new_odom);
    }
    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }
    void smooth_yaw(void)
    {
        double dyaw = xref(3, 0) - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            xref(3, 0) -= M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            xref(3, 0) += M_PI * 2;
            dyaw = xref(3, 0) - now_state.theta;
        }

        for (int i=0; i<T-1; i++)
        {
            dyaw = xref(3, i+1) - xref(3, i);
            while (dyaw >= M_PI / 2)
            {
                xref(3, i+1) -= M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
            while (dyaw <= -M_PI / 2)
            {
                xref(3, i+1) += M_PI * 2;
                dyaw = xref(3, i+1) - xref(3, i);
            }
        }
    }
    double get_nearest(void)
    {
        Eigen::Vector3d track_point = csp.get_state(t_track);
        Eigen::Vector2d now_point(now_state.x, now_state.y);
        double min_dist = (now_point-track_point.head(2)).norm();
        double track_temp = t_track;
        double interval = now_state.v * dt;
        
        for (double t_temp = track_temp + interval, i=1.0; i < 10; i += 1.0, t_temp += interval)
        // for (double t_temp = track_temp + 1.0, i=1.0; i < 10; i += 1.0, t_temp += 1.0)
        {
            if (t_temp > traj_duration_)
            {
                t_temp = traj_duration_;
            }
            Eigen::Vector3d temp = csp.get_state(t_temp);
            double dist = (temp.head(2) - now_point).norm();
         
            if (dist<min_dist)
            {
                min_dist = dist;
                t_track = t_temp;
            }
        }
        // ROS_INFO("t_track=%f,traj_duration=%f",t_track,traj_duration_);

        return t_track;
    }
    void drawFollowPath(void)
    {
        int id = 0;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;
        id++;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = 0.3;
        sphere.scale.y = 0.3;
        sphere.scale.z = 0.3;
        line_strip.scale.x = 0.3 / 2;
        geometry_msgs::Point pt;
        
        for (auto p:csp_path)
        {
            pt.x = p(0);
            pt.y = p(1);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        vis_pub.publish(line_strip);
    }

public:
	MPC() : csp(vector<double>{0.0, 30.0, 6.0, 20.0, 35.0, 10.0, 0.0, 0.0}, vector<double>{0.0, 0.0, 20.0, 35.0, 20.0, 30.0, 5.0, 0.0}, 1.0) {}
    void init(ros::NodeHandle &nh);
	~MPC() {}
};