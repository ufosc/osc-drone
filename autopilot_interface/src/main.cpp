#include "autopilot_interface/autopilot_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "autopilot_interface_node");

    AutopilotInterface autopilot;

    autopilot.armVehicle(true);
    
    ros::spin();
    return 0;
}
