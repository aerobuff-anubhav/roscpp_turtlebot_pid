//
// Created by anubhav on 5/11/21.
//
#include "../include/controller.hpp"

namespace roscpp_turtlebot_pid{
    Controller::Controller(ros::NodeHandle nh_private, ros::NodeHandle nh_public){
        nh_private.getParam("sim", sim);
        nh_private.getParam("mocap", mocap);
        nh_private.getParam("Kp_angular", Kp_angular);
        nh_private.getParam("Kp_linear", Kp_linear);
        nh_private.getParam("vel_x_max", vel_x_max);
        nh_private.getParam("yaw_rate_max", yaw_rate_max);
        nh_private.getParam("safety_distance", safety_distance);

        publisher_cmd_vel = nh_public.advertise<geometry_msgs::Twist>("pub_info", 1000);
        if (sim==true) {
            subscriber_agent_pose = nh_public.subscribe("agent_pose", 1000, &Controller::agent_pose_simcallback, this);
        }
        else{
            subscriber_agent_pose = nh_public.subscribe("agent_pose", 1000, &Controller::agent_pose_callback, this);
        }
        subscriber_target_pose = nh_public.subscribe("target_pose", 1000, &Controller::target_pose_callback, this);
    }

    void Controller::agent_pose_simcallback(const nav_msgs::Odometry &msg){
        agent_pose.pose.position.x = msg.pose.pose.position.x;
        agent_pose.pose.position.y = msg.pose.pose.position.y;
        agent_pose.pose.position.z = msg.pose.pose.position.z;
        agent_pose.pose.orientation.x = msg.pose.pose.orientation.x;
        agent_pose.pose.orientation.y = msg.pose.pose.orientation.y;
        agent_pose.pose.orientation.z = msg.pose.pose.orientation.z;
        agent_pose.pose.orientation.w = msg.pose.pose.orientation.w;
    }

    void Controller::agent_pose_callback(const geometry_msgs::PoseStamped &msg){
        agent_pose = msg;
    }

    void Controller::target_pose_callback(const geometry_msgs::PoseStamped &msg){
        target_pose = msg;
    }

    void Controller::command_velocity() {
        double target_x = target_pose.pose.position.x;
        double target_y = target_pose.pose.position.y;
        double target_z = target_pose.pose.position.z;
        double target_q0 = target_pose.pose.orientation.x;
        double target_q1 = target_pose.pose.orientation.y;
        double target_q2 = target_pose.pose.orientation.z;
        double target_q3 = target_pose.pose.orientation.w;

        double agent_x = agent_pose.pose.position.x;
        double agent_y = agent_pose.pose.position.y;
        double agent_z = agent_pose.pose.position.z;
        double agent_q0 = agent_pose.pose.orientation.x;
        double agent_q1 = agent_pose.pose.orientation.y;
        double agent_q2 = agent_pose.pose.orientation.z;
        double agent_q3 = agent_pose.pose.orientation.w;

        Eigen::Matrix<double, 4, 1> target_q;
        target_q << target_q0, target_q1, target_q2, target_q3;
        Eigen::Matrix<double, 3, 1> target_e;
        target_e = reef_msgs::fromQuaternionToEulerAngle<Eigen::Matrix<double, 4, 1>, Eigen::Matrix<double, 3, 1>>(
                target_q);

        Eigen::Matrix<double, 4, 1> agent_q;
        agent_q << agent_q0, agent_q1, agent_q2, agent_q3;
        Eigen::Matrix<double, 3, 1> agent_e;
        agent_e = reef_msgs::fromQuaternionToEulerAngle<Eigen::Matrix<double, 4, 1>, Eigen::Matrix<double, 3, 1>>(
                agent_q);

        Eigen::Matrix<double, 3, 1> target_linear_position;
        target_linear_position << target_x, target_y, target_z;

        Eigen::Matrix<double, 3, 1> agent_linear_position;
        agent_linear_position << agent_x, agent_y, agent_z;

        double relative_yaw = atan2((target_y - agent_y), (target_x - agent_x));
        double angular_error = relative_yaw - agent_e[0];
        double distance_euclidean = (target_linear_position - agent_linear_position).squaredNorm();

        if (angular_error > M_PI)
            angular_error = angular_error - 2 * M_PI;
        if (angular_error < -M_PI)
            angular_error = angular_error + 2 * M_PI;

        double cmd_vel_angular = Kp_angular * angular_error;
        double cmd_vel_linear = Kp_linear * distance_euclidean;

        if (cmd_vel_linear > vel_x_max) {
            cmd_vel_linear = vel_x_max;
        }

        if (cmd_vel_angular > yaw_rate_max) {
            cmd_vel_angular = yaw_rate_max;
        }

        if (cmd_vel_angular < -yaw_rate_max) {
            cmd_vel_angular = -yaw_rate_max;
        }

        if (distance_euclidean < safety_distance) {
            cmd_vel_linear = 0;
            cmd_vel_angular = 0;
        }

        msg.linear.x = cmd_vel_linear;
        msg.angular.z = cmd_vel_angular;

        publisher_cmd_vel.publish(msg);
    }
}