//
// Created by anubhav on 5/11/21.
//

#ifndef ROSCPP_TURTLEBOT_PID_CONTROLLER_HPP
#define ROSCPP_TURTLEBOT_PID_PCONTROLLER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Core"
#include <reef_msgs/ReefMsgsConversionAPI.h>
#include "math.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "sstream"

namespace roscpp_turtlebot_pid{
    class Controller{
    private:
        ros::NodeHandle nh_private;
        ros::Publisher publisher_cmd_vel;
        ros::Subscriber subscriber_agent_pose;
        ros::Subscriber subscriber_target_pose;

        int count=0;
        bool sim;
        bool mocap;
        double vel_x_max;
        double Kp_angular;
        double Kp_linear;
        double yaw_rate_max;
        double safety_distance;

        geometry_msgs::Twist msg;
        geometry_msgs::PoseStamped agent_pose;
        geometry_msgs::PoseStamped target_pose;

    public:
        ros::NodeHandle nh_public;
        Controller(ros::NodeHandle nh_private, ros::NodeHandle nh_public);
        void command_velocity();
        void agent_pose_simcallback(const nav_msgs::Odometry &_poseMsg);
        void agent_pose_callback(const geometry_msgs::PoseStamped &_poseMsg);
        void target_pose_callback(const geometry_msgs::PoseStamped &_poseMsg);
//        ~Controller();
    };
}

#endif //ROSCPP_TURTLEBOT_PID_CONTROLLER_HPP
