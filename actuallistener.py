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
def speed_callback(self, name,value):
    print(value)
    print("-------")
    return value

vehicle.add_attribute_listener("groundspeed", speed_callback)


#vehicle.add_attribute_listener("location.global_frame", location_callback)
j=0
while True:
    inittime = time.time()
    while j < 5:
        print ("here")
        i = 0
        sum = 0
        while  i < 100:
            #time1 = time.time()
            #print (speed_callback, (time.time()-inittime))
            print(speed_callback)
            #time2 = time.time()
            #sum = sum + (time2-time1)
            i+=1
        #time.sleep(0.5)
        j +=1
   # print(sum/i)