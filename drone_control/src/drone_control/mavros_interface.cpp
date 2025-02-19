#include "../../include/drone_control/mavros_interface.h"
#include <mavros_msgs/CommandBool.h>

MavrosInterface::MavrosInterface() {
    // Initialize the clients
    this->armingClient = this->nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

/*

Sets the arm state of the drone to true or false, depending on the parameter.

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
