//
// Created by ros on 20.05.2020.
//

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <myrobot_custom_nav/Yuk.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.5;
    goal.target_pose.pose.position.y = 8.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.z = 1.57;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

        ROS_INFO("Call the camera navigation service");


        ros::ServiceClient client = n.serviceClient<myrobot_custom_nav::Yuk>("yuk_kaldir");
        myrobot_custom_nav::Yuk srv;
        srv.request.Reqs = true;
        if (client.call(srv))
        {
            ROS_INFO("Done");
        }
        else
        {
            ROS_ERROR("Failed to call service YUK");
            return 1;
        }

    }
    else
        ROS_INFO("The base failed to reach desired point");

    return 0;
}