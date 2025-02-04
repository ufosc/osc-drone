#!/usr/bin/env python3

import rospy
from drone_health_monitor.msg import DroneHealth
from mavros_msgs.msg import State

class DroneHealthMonitor():
    def __init__(self):
        rospy.init_node("drone_health_monitor")

        # Print at 1 Hz looped
        self.rate = 1.0
        self.loop_rate = rospy.Rate(self.rate)

        self.drone_health = DroneHealth()
        
        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def state_callback(self, msg: State) -> None:
        self.drone_health.system_ok = msg.connected
        self.drone_health.system_mode = msg.mode
        self.drone_health.fcu_connected = msg.connected

    def display_drone_health(self) -> None:
        rospy.loginfo(f"Drone FCU Connected: { self.drone_health.fcu_connected }")

    def start(self) -> None:
        while not rospy.is_shutdown():
            self.display_drone_health()
            self.loop_rate.sleep()


if __name__ == "__main__":
    drone_health = DroneHealthMonitor()
    drone_health.start()

    

    

