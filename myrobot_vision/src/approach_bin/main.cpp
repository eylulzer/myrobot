//
// Created by ros on 12.05.2020.
//

#include <ros/ros.h>
#include "cv_approach_bin.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "opencv_nav");
    ApproachBin approachBin;
    approachBin.init();

    ros::spin();
    return 0;
}