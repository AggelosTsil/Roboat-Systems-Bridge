# Connect to vehicle
vehicle = connect('/dev/ttyACM0', wait_ready=True,baud=57600)

# Get the mission plan 
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Save the vehicle commands to a list
missionlist=[]
for cmd in cmds:
    missionlist.append(cmd)

# Example of single command modification
missionlist[0].command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF 

# --Clear the current mission (command is sent when we call upload())
cmds.clear()

# --Write the modified mission and flush to the vehicle
for cmd in missionlist:
    cmds.add(cmd)
cmds.upload()

# End of modification

# Example of modification mid-mission
x = 10 #latitude
y = 10 #longitude
missionlist[vehicle.commands.next].command=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x, y, 0)

# --Clear the current mission (command is sent when we call upload())
cmds.clear()

# --Write the modified mission and flush to the vehicle
for cmd in missionlist:
    cmds.add(cmd)
cmds.upload()

# End of modification

