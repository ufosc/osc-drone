#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>


class AutopilotInterface {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient arming_client;
        ros::ServiceServer arm_service;

        ros::ServiceClient mode_client;
        ros::ServiceServer mode_service;
        
    public:
        AutopilotInterface();

        bool armVehicle(bool arm);

        bool armCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        bool setMode(int mode);

        bool setModeCallback(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res);
};
