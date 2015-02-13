"""
Launch file to start all processes for thesis demo.
"""

from collections import deque
import spur
import time, sys, os, signal

start = time.time()

# set up shells
squirrel = spur.SshShell(hostname="10.9.160.238", 
                         username="squirrel", 
                         password="asdf")
asdf = spur.SshShell(hostname="10.8.190.94", 
                     username="asdf", 
                     password="asdf")
 
# set up a queue of commands to execute and processes launched
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

processQueue = deque()

try:

    # launch processes in order
    for task in commandQueue:
        command = task["command"]
        shell = task["host"]
        
        process = shell.spawn(command,
                              stdout=sys.stdout, 
                              store_pid=True)
        
        processQueue.append(process)
        print str(command) + " pid: " + str(process.pid)
        

        # wait until each command is done
        time.sleep(10) # for now, just wait 10 seconds


    # wait for keyboard interrupt, and then kill all processes
    while True:
        time.sleep(1)
        
except KeyboardInterrupt:
    while len(processQueue) > 0:
        processQueue.pop().send_signal(signal.CTRL_C_EVENT)

    print "All processes shut down cleanly."
    sys.exit()
