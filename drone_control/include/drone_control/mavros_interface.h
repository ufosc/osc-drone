#pragma once

#include "ros/node_handle.h"
#include "ros/service_client.h"
#include <iostream>
#include <ros/ros.h>

enum class Mode {
    STABILIZE,  // Manual control with leveling assistance
    ACRO,       // Full manual control without stabilization
    ALT_HOLD,   // Holds altitude with manual control for pitch/roll/yaw
    AUTO,       // Executes mission from flight plan
    GUIDED,     // Controlled by GCS or companion computer
    LOITER,     // Holds position using GPS
    RTL,        // Returns to launch (home) position
    LAND,       // Lands the vehicle
    TAKEOFF,    // Automatic takeoff
    POSHOLD,    // Position hold with pilot override
    CIRCLE,     // Circles around a point
    DRIFT,      // Assisted mode for smooth flight
    SPORT,      // High-speed mode with stabilization
    BRAKE,      // Stops movement immediately using GPS
    THROW,      // Allows launching the drone by throwing
    AVOID_ADSB, // Avoidance mode for ADS-B equipped aircraft
    GUIDED_NOGPS // Guided mode without GPS (indoor use)
};

class MavrosInterface {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient armingClient;
        ros::ServiceClient modeClient;

    public:
        MavrosInterface();

        bool setArmed(bool arm);

        bool changeMode(Mode mode);
};
