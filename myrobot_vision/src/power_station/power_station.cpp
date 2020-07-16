//
// Created by melodic on 7/14/20.
//

#include <ros/ros.h>
#include "Docking.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "power_station");

    Docking docker;
    docker.init();

    ros::spin();

    return EXIT_SUCCESS;
}
