//
// Created by ros on 12.05.2020.
//

#include <ros/ros.h>
#include "cv_approach_bin.h"
#include "myrobot_vision/Yuk.h"

bool isRunning = false;

bool respond(myrobot_vision::Yuk::Request  &req,
             myrobot_vision::Yuk::Response &res)
{

    ROS_INFO_STREAM("REQUEST: " << req.Reqs << std::endl);

    res.Resps = true;
    isRunning = true;

    sleep(20);

    return res.Resps;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_nav");

    ros::NodeHandle n;

    ApproachBin approachBin;

    ros::ServiceServer service = n.advertiseService("yuk_kaldir", respond);
    ROS_INFO("Ready to take load.");

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        if(isRunning){
            approachBin.init();
            break;
        }

        ros::spinOnce();
        r.sleep();
    }

    while (ros::ok())
    {
        if(approachBin.isDone){
            isRunning = false;
            break;
        }

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}

