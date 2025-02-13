#include "../include/autopilot_interface/autopilot_interface.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"

/*
@brief AutopilotInterface class constructor

This is the constructor for the AutopilotInterface class.
It assigns and allocates variables and resources before use.
*/
AutopilotInterface::AutopilotInterface() {
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    this->arm_service = nh.advertiseService("/autopilot/arm", &AutopilotInterface::armCallback, this);

    this->mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    this->mode_service = nh.advertiseService("/autopilot/set_mode", &AutopilotInterface::setModeCallback, this);
}


/*
@brief Arms or disarms the drone

@param arm: A boolean where true means arm and false means disarm
@return bool: true if success, false if failure
*/
bool AutopilotInterface::armVehicle(bool arm) {
    mavros_msgs::CommandBool srv;
    srv.request.value = arm;

    if (arming_client.call(srv) && srv.response.success) {
        ROS_INFO_STREAM((arm ? "Armed" : "Disarmed") << " successfully!");
        return true;
    } else {
        ROS_INFO_STREAM("Failed to " << (arm ? "Arm." : "Disarm."));
        return false;
    }
}

/*
@brief Callback for arm service

@param req: Reference to incoming request, res: Reference to incoming res (which we modify so requester can see the result)
@return bool: true to signal finish
*/
bool AutopilotInterface::armCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    res.success = armVehicle(req.data);

    if (req.data) {
        res.message = res.success ? "Armed successfully" : "Failed to arm.";
    } else {
        res.message = res.success ? "Disarmed successfully" : "Failed to disarm";
    }

    return true;
}

/*
@brief Sets the mode of the drone

@param mode: integer that corresponds to a specific mode
@return bool: true if success, false if failure
*/
bool AutopilotInterface::setMode(int mode) {
    mavros_msgs::SetMode srv;

    srv.request.base_mode = mode;
    if (mode_client.call(srv) && srv.response.mode_sent) {
        ROS_INFO_STREAM("Successfully set mode!");
        return true;
    } else {
        ROS_INFO_STREAM("Failed to set mode!");
        return false;
    }
}

bool AutopilotInterface::setModeCallback(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res) {
    res.mode_sent = setMode(req.base_mode);
    return true;
}
