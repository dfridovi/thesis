"""
Launch file to start all processes for thesis demo.
Uses spur library, which is a high-level API on top of Paramiko.

Works fine except it does not run in "login" mode. This means
that it does not automatically login with the correct environment
variables, and does not behave correctly in conjunction with processes
(e.g. ROS nodes) on other machines.
"""

from collections import deque
import spur
import time, sys, os
import cPickle as pickle

from env_setup import hardCodeSquirrelEnv, hardCodeAsdfEnv


start = time.time()

# set up shells
squirrel = spur.SshShell(hostname="10.9.160.238", 
                         username="squirrel", 
                         password="asdf")
asdf = spur.SshShell(hostname="10.8.190.94", 
                     username="asdf", 
                     password="asdf")
 
# set up environments
squirrelEnv = None
if os.path.exists("./squirrelEnv.pkl"):
    file = open("./squirrelEnv.pkl", "rb")
    squirrelEnv = pickle.load(file)
    file.close()
else: 
    squirrelEnv = hardCodeSquirrelEnv()

asdfEnv = None
if os.path.exists("./asdfEnv.pkl"):
    file = open("./asdfEnv.pkl", "rb")
    asdfEnv = pickle.load(file)
    file.close()
else: 
    asdfEnv = hardCodeAsdfEnv()

# set up a queue of commands to execute and processes launched
commandQueue = deque()
commandQueue.append({"command" : ["roscore"],
                     "host"    : squirrel})


"""
commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_bringup", 
                                  "minimal.launch"],
                     "host"    : asdf})


commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_navigation", 
                                  "gmapping_demo.launch"],
                     "host"    : asdf})
commandQueue.append({"command" : ["roslaunch",
                                  "turtlebot_rviz_launchers",
                                  "view_navigation.launch"],
                     "host"    : asdf})
"""

processQueue = deque()

try:

    # launch processes in order
    for task in commandQueue:
        command = task["command"]
        shell = task["host"]
        
        process = shell.spawn(command,
                              update_env=asdfEnv,
                              stdout=sys.stdout,
                              store_pid=True)
        
        processQueue.append({"process" : process,
                             "command" : command,
                             "host"    : shell})
        print str(command) + " pid: " + str(process.pid)
        

        # wait until each command is done
        time.sleep(10) # for now, just wait 10 seconds


    # wait for keyboard interrupt, and then kill all processes
    while True:
        time.sleep(1)
        
except KeyboardInterrupt:
    while len(processQueue) > 0:
        task = processQueue.pop()
        print "Terminating " + str(task["command"])

        shell = task["host"]
#        process = shell.run(["kill", "-15", str(task["process"].pid)],
#                            update_env=asdfEnv,
#                            stdout=sys.stdout)

    print "All processes shut down cleanly."
    sys.exit()


