//
// Created by ros on 12.05.2020.
//

#include <ros/ros.h>
#include "cv_approach_bin.h"
#include "myrobot_vision/Yuk.h"
#include <thread>

ApproachBin approachBin;

void waitForBin(){

    while(!approachBin.isDone)
        ros::Duration(1).sleep();
}

bool respond(myrobot_vision::Yuk::Request  &req,
             myrobot_vision::Yuk::Response &res)
{

    ROS_INFO_STREAM("REQUEST: " << req.Reqs << std::endl);

    res.Resps = true;

    approachBin.init();

    std::thread th1(waitForBin);
    th1.join();

    return res.Resps;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_nav");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("yuk_kaldir", respond);
    ROS_INFO("Ready to take load.");

    ros::spin();

    return 0;
}

