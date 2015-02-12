"""
Launch file to start all process for thesis demo.
"""

import spur
import time, sys, os

start = time.time()

# set up shell
shell = spur.SshShell(hostname="10.8.244.74", 
                      username="pi", 
                      password="raspberry") 
command = ["python", "-u", "test_stdout.py"]
path = "/home/pi/thesis/LoadManager"

# launch process
process = shell.spawn(command,
                      cwd=path,
                      stdout=sys.stdout, 
                      store_pid=True)

print process.pid
#print process.output
message = "[demo_launch] Waiting... Elapsed time: "
while True:
    time.sleep(5)
    print message + str(int(time.time() - start)) + " seconds"
    
