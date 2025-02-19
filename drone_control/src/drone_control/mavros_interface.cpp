#include "../../include/drone_control/mavros_interface.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

MavrosInterface::MavrosInterface() {
    // Initialize the clients
    this->armingClient = this->nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    this->modeClient = this->nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

/*

Sets the arm state of the drone to true or false, depending on the parameter.

@note Will return false if MODE is set to LAND or any other mode that can't be armed

@returns true - successfully set, false - something failed
*/
bool MavrosInterface::setArmed(bool arm) {
    mavros_msgs::CommandBool arming_cmd;

    arming_cmd.request.value = arm;

    if (armingClient.call(arming_cmd)) {
        if (arming_cmd.response.success) {
            // Return true if service call was successful
            return true;
        }
    }

    // Return false if any other case that is NOT successful
    return false;
}

bool MavrosInterface::changeMode(Mode mode) {
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.base_mode = 0;

    switch (mode) {
        case Mode::LAND:
            set_mode_srv.request.custom_mode = "LAND";
            break;
        case Mode::GUIDED:
            set_mode_srv.request.custom_mode = "GUIDED";
        default:
            ROS_ERROR("Error: Mode requested was not a valid mode.");
            return false;
    }

    if (this->modeClient.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
        return true;
    } else {
        return false;
    }
}
