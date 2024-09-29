from pymavlink import mavutil
import time

#========================================================================================================================
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

while True:
    # Ask the user for the x direction distance
    try:
        x_distance = float(input("Enter the x direction distance: "))
    except ValueError:
        print("Invalid input. Please enter a numerical value.")
        continue
    
    # Send the command with the user-provided x direction distance
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, 0b110111111000,
        x_distance, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    ))
    
    # Add a small delay to avoid sending messages too rapidly
    time.sleep(1)
