"""
Launch file to start all process for thesis demo.
"""

import spur
import time

shell = spur.LocalShell()
minimum_launch = shell.spawn(["roslaunch", "turtlebot_bringup", "minimal.launch"], 
                             stdout=1, store_pid=True)

print minimum_launch.pid
minimum_launch.wait_for_result()
    

