#pragma once

#include "ros/node_handle.h"
#include "ros/service_client.h"
#include <iostream>
#include <ros/ros.h>

class MavrosInterface {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient armingClient;

    public:
        MavrosInterface();

        bool setArmed(bool arm);
};
