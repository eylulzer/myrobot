#ifndef MYROBOT_HARDWARE_INTERFACE_H
#define MYROBOT_HARDWARE_INTERFACE_H
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <iostream>
#include <cmath>

class Myrobot : public hardware_interface::RobotHW {

private:

    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    double cmd[2], pose[2], vel[2], eff[2];

public:

    Myrobot(){

        cmd[0] = 0, cmd[1] = 0, pose[0] = 0, pose[1] = 0;
        vel[0] = 0, vel[1] = 0, eff[0] = 0, eff[1] = 0;
        

        hardware_interface::JointStateHandle left_state_handle("left_wheel_joint", &pose[0], &vel[0], &eff[0]); //durum pozisyon hız bilgisi 
        hardware_interface::JointStateHandle right_state_handle("right_wheel_joint", &pose[1], &vel[1], &eff[1]);
        joint_state_interface.registerHandle(left_state_handle);
        joint_state_interface.registerHandle(right_state_handle);
        hardware_interface::JointHandle left_handle(joint_state_interface.getHandle("left_wheel_joint"), &cmd[0]); //cmd_veli ayrı tekerler için komutu 
        hardware_interface::JointHandle right_handle(joint_state_interface.getHandle("right_wheel_joint"), &cmd[1]); //cmd_veli ayrı tekerler için komutu 
        registerInterface(&joint_state_interface);
        position_joint_interface.registerHandle(left_handle);
        position_joint_interface.registerHandle(right_handle);
        velocity_joint_interface.registerHandle(left_handle);
        velocity_joint_interface.registerHandle(right_handle);
        effort_joint_interface.registerHandle(left_handle);
        effort_joint_interface.registerHandle(right_handle);
        registerInterface(&effort_joint_interface);
        registerInterface(&velocity_joint_interface);
        registerInterface(&position_joint_interface);
    }

    virtual ~Myrobot() {

    };

    void read(ros::Time time, const ros::Duration &period){
        pose[0] = pose[0] + (cmd[0] * period.toSec()); //2. kısım angular yol 
        pose[1] = pose[1] + (cmd[1] * period.toSec());

        vel[0] = cmd[0];
        vel[1] = cmd[1];

    };
    
    void write(ros::Time time, const ros::Duration &period){
        
    };

};

#endif //ROS_WORKSPACE_BIRFENAGV_HARDWARE_INTERFACE_H