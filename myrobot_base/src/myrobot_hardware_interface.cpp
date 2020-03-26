#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>
#include "myrobot_base/myrobot_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myrobothw");
    ros::NodeHandle nh;
    ros::CallbackQueue q;

    Myrobot robot;

    nh.setCallbackQueue(&q);
    controller_manager::ControllerManager cm(&robot, nh);
    ros::AsyncSpinner asy(4, &q);
    asy.start();
    
    ros::Time prev_time = ros::Time::now();
    ros::Rate m_rate(40);

    ros::Time t;
    ros::Duration d;

    while (ros::ok()) {

        t = ros::Time::now();
        d = t - prev_time;

        prev_time = t;
        
        robot.read(t, d);
        cm.update(t, d);
        robot.write(t, d);

        m_rate.sleep();
    }
    
    asy.stop();
    return 0;
}