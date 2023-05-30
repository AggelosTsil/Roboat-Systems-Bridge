# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print ("Connecting")
connection_string = '/dev/ttyUSB0' #doesn't get it regardless of USB port
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Display basic vehicle state
print (" Type: %s" % vehicle._vehicle_type)
print (" Armed: %s" % vehicle.armed)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
