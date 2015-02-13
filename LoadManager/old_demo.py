"""
Outdated demo launch file.
"""

import spur
import time, sys, os

start = time.time()

# set up shell
squirrel = "10.9.160.238"
asdf = "10.8.190.94" 
shell = spur.SshShell(hostname=asdf,
                      username="asdf", 
                      password="asdf") 
#setup = ["source", "/home/asdf/.bashrc"]
#command = ["roscore"]
#command = ["roslaunch", "turtlebot_bringup", "minimal.launch"]
command = ["printenv"]

# launch process
#init = shell.run(setup)
#print init.output

process = shell.spawn(command,
                      stdout=sys.stdout, 
                      store_pid=True)
print process.pid

message = "[demo_launch] Waiting... Elapsed time: "
while True:
    time.sleep(5)
    print message + str(int(time.time() - start)) + " seconds"
