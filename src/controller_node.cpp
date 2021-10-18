//
// Created by anubhav on 5/11/21.
//

#include "../include/controller.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public("");
    ros::Rate loop_rate(10);

    roscpp_turtlebot_pid::Controller controller_object(nh_private, nh_public);

    while(ros::ok()){
        controller_object.command_velocity();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}