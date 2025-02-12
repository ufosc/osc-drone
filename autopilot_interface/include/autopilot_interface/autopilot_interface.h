#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <mavros_msgs/CommandBool.h>


class AutopilotInterface {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient arming_client;
        ros::ServiceServer arm_service;
        
    public:
        AutopilotInterface();

        bool armVehicle(bool arm);

        bool armCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};
