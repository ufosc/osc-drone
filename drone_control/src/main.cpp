#include <iostream>
#include "../include/drone_control/mavros_interface.h"
#include "ros/init.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "drone_control_node");

    MavrosInterface mavrosInterface;

    if (mavrosInterface.setArmed(true)) {
        ROS_INFO("Successfully armed client!");
    } else {
        ROS_ERROR("Something went wrong with the arming client");
    }
    
    return 0;
}
