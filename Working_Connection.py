# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print ("Connecting")
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Display basic vehicle state
print (" Type: %s" % vehicle._vehicle_type)
print (" Armed: %s" % vehicle.armed)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
print (" Mode: %s" % vehicle.mode.name)

missionlist=[]
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
for cmd in cmds:
    missionlist.append(cmd)
print (missionlist)