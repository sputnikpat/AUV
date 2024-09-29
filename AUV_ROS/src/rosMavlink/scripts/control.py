#!/usr/bin/env python

import rospy
from pymavlink import mavutil
import time
import signal
import sys

def signal_handler(sig, frame):
    rospy.loginfo("Signal received, disarming the vehicle...")
    if 'master' in globals():
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        master.motors_disarmed_wait()
        rospy.loginfo("Disarmed!")
    sys.exit(0)

def main():
    rospy.init_node('drone_control_node', anonymous=True)

    # Create the connection
    global master
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    
    # Wait for a heartbeat before sending commands
    master.wait_heartbeat()

    # Arm the vehicle
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # Wait until arming confirmed
    rospy.loginfo("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    rospy.loginfo("Armed!")

    # Set mode to ALT_HOLD
    rospy.loginfo("Putting in ALT_HOLD mode")
    DEPTH_HOLD = 'ALT_HOLD'  # For pixhawk use ALT_HOLD, for sim use GUIDED
    DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        master.set_mode(DEPTH_HOLD)
    rospy.loginfo("ALT_HOLD mode ON!")

    # Define movement functions
    def dive(alt):
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, 
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b110111111000, 
            0, 0, alt, 0, 0, 0, 0, 0, 0, 0, 0))
        rospy.loginfo("Dive target accomplished")

    def forward(distx):
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, 
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b110111111000, 
            distx, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        rospy.loginfo("Forward target accomplished")

    def turn(yaw):
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, 
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b100111111000, 
            0, 0, 0, 0, 0, 0, 0, 0, 0, yaw, 0))
        rospy.loginfo("Turn target accomplished")

    while not rospy.is_shutdown():
        # Perform a sequence of movements
        forward(10)
        dive(-5)
        turn(45)
        forward(5)
        turn(-45)
        dive(5)
        rospy.loginfo("Completed one cycle of movements")
        time.sleep(10)  # Adjust the sleep duration as needed

    # Disarm the vehicle when rospy is shutting down
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # Wait until disarming confirmed
    rospy.loginfo("Waiting for the vehicle to disarm")
    master.motors_disarmed_wait()
    rospy.loginfo("Disarmed!")
    rospy.loginfo("End of mission")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
