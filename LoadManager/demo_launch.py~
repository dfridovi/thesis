"""
Launch file to start all processes for thesis demo.
"""

from collections import deque
import spur
import time, sys, os

start = time.time()

# set up shells
squirrel = spur.SshShell(hostname="10.9.160.238", 
                         username="squirrel", 
                         password="asdf")
asdf = spur.SshShell(hostname="10.8.190.94", 
                     username="asdf", 
                     password="asdf")
 
# set up a queue of commands to execute
commandQueue = deque()
commandQueue.append({"command" : ["roscore"],
                     "host"    : squirrel})
commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_bringup", 
                                  "minimum.launch"],
                     "host"    : asdf})
commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_navigation", 
                                  "gmapping_demo.launch"],
                     "host"    : asdf})
commandQueue.append({"command" : ["roslaunch",
                                  "turtlebot_rviz_launchers",
                                  "view_navigation.launch"],
                     "host"    : asdf})

# launch processes in order
for task in commandQueue:
    command = task["command"]
    shell = task["host"]

    process = shell.spawn(command,
                          stdout=sys.stdout, 
                          store_pid=True)
    print str(command) + " pid: " + str(process.pid)

    # wait until each command is done
    time.sleep(10) # for now, just wait 10 seconds


#print process.output
message = "[demo_launch] Waiting... Elapsed time: "
while True:
    time.sleep(5)
    print message + str(int(time.time() - start)) + " seconds"
    
