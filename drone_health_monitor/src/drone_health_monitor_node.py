#!/usr/bin/env python3

import rospy
from drone_health_monitor.msg import DroneHealth
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState, Temperature

class DroneHealthMonitor():
    def __init__(self):
        rospy.init_node("drone_health_monitor")

        self.drone_health = DroneHealth()
        
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.global_position_callback)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_state_callback)
        rospy.Subscriber("/mavros/imu/temperature", Temperature, self.imu_temperature_callback)

        # Publisher to /drone_health
        self.pub = rospy.Publisher("/drone_health", DroneHealth, queue_size=10)

        # Print at 10 Hz looped
        self.rate = 10
        self.loop_rate = rospy.Rate(self.rate)

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

    def imu_temperature_callback(self, msg: Temperature):
        self.drone_health.temperature = msg.temperature

    # This is for debugging while developing
    def display_drone_health(self) -> None:
        rospy.loginfo(f"{ self.drone_health }")

    def start(self) -> None:
        while not rospy.is_shutdown():
            self.drone_health.header.stamp = rospy.Time.now()
            self.pub.publish(self.drone_health)
            self.loop_rate.sleep()


if __name__ == "__main__":
    drone_health = DroneHealthMonitor()
    drone_health.start()

    

    

