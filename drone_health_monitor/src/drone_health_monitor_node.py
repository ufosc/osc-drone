#!/usr/bin/env python3

import rospy
from drone_health_monitor.msg import DroneHealth
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState

class DroneHealthMonitor():
    def __init__(self):
        rospy.init_node("drone_health_monitor")

        # Print at 1 Hz looped
        self.rate = 1.0
        self.loop_rate = rospy.Rate(self.rate)

        self.drone_health = DroneHealth()
        
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.global_position_callback)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_state_callback)

    def state_callback(self, msg: State) -> None:
        self.drone_health.header = msg.header
        self.drone_health.is_armed = msg.armed
        self.drone_health.is_connected = msg.connected
        self.drone_health.flight_mode = msg.mode

    def global_position_callback(self, msg: NavSatFix):
        self.drone_health.gps_ok = msg.status.status >= msg.status.STATUS_FIX

    def battery_state_callback(self, msg: BatteryState):
        self.drone_health.battery_percent = msg.percentage
        self.drone_health.battery_voltage = msg.voltage

    def display_drone_health(self) -> None:
        rospy.loginfo(f"Drone Battery Connected: { self.drone_health.battery_percent }")

    def start(self) -> None:
        while not rospy.is_shutdown():
            self.display_drone_health()
            self.loop_rate.sleep()


if __name__ == "__main__":
    drone_health = DroneHealthMonitor()
    drone_health.start()

    

    

