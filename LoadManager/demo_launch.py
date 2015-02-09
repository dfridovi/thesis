"""
Launch file to start all process for thesis demo.
"""

import subprocess
import spur
import time, sys

start = time.time()
shell = spur.LocalShell()
command = ["python", "test_stdout.py"]
process = shell.spawn(command, stdout=sys.stdout, store_pid=True)

print process.pid
message = "[demo_launch] Waiting... Elapsed time: "
while True:
    time.sleep(5)
    print message + str(int(time.time() - start)) + " seconds"
    
