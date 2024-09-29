#!/usr/bin/env python3

from pymavlink import mavutil
import time
import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
    x_distance, y_distance, z_distance = data.data

    # Send the command with the user-provided x, y, z distances
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b110111111000,
        x_distance, y_distance, z_distance, 0, 0, 0, 0, 0, 0, 0, 0
    ))

    # Add a small delay to avoid sending messages too rapidly
    time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('mavlink_node', anonymous=True)

    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    rospy.Subscriber("distance_data", Float32MultiArray, callback)
    rospy.spin()

