"""
Launch file to start all processes for thesis demo.
Uses the psutil library to launch processes (as a wrapper for
subprocess) and also monitor CPU utilization.

This file launches ssh interactive login shells on each host and 
sets up processes on each of them by sending commands over stdin.
"""

import psutil, subprocess
import time, sys, os


# set up shells
(RPI_USR, RPI_IP) = ("pi", "10.8.244.74")
SQUIRREL = "squirrel"
ASDF = "asdf"

def openSSH(usr, ip):
   
    SSH = ["ssh", "-t", "-t", usr + "@" + ip]
    ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
    return ssh

def navigation():

    ROSCORE = "roscore"
    MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch"
    SENSE_LAUNCH = "roslaunch turtlebot_bringup 3dsensor_edited.launch"
    AMCL_LAUNCH = "roslaunch turtlebot_navigation amcl_demo_edited.launch map_file:=/tmp/ee_lab_big.yaml"

    roscore = openSSH(SQUIRREL, SQUIRREL)
    roscore.stdin.write(ROSCORE + "\n")
    time.sleep(5)
    
    minlaunch = openSSH(ASDF, ASDF)
    minlaunch.stdin.write(MIN_LAUNCH + "\n")
    time.sleep(5)
    
    senselaunch = openSSH(ASDF, ASDF)
    senselaunch.stdin.write(SENSE_LAUNCH + "\n")
    time.sleep(5)

    amcllaunch = openSSH(SQUIRREL, SQUIRREL)
    amcllaunch.stdin.write(AMCL_LAUNCH + "\n")

def mapping():

    PRINTENV = "printenv"
    ROSCORE = "roscore"
    MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch"
    MAP_LAUNCH = "roslaunch turtlebot_navigation gmapping_demo.launch"

    # "roslaunch turtlebot_rviz_launchers view_navigation.launch"

    roscore = openSSH(SQUIRREL, SQUIRREL)
    roscore.stdin.write(ROSCORE + "\n")
    time.sleep(5)
    
    minlaunch = openSSH(ASDF, ASDF)
    minlaunch.stdin.write(MIN_LAUNCH + "\n")
    time.sleep(5)
    
    maplaunch = openSSH(ASDF, ASDF)
    maplaunch.stdin.write(MAP_LAUNCH + "\n")


def main():

    try:
        mapping()
#        navigation()

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
