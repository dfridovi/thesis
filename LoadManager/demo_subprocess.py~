"""
Launch file to start all processes for thesis demo.
Uses basic subprocess library to launch commands on the local shell.

This file launches ssh interactive login shells on each host and 
sets up processes on each of them by sending commands over stdin.
"""

import subprocess
import time, sys, os

# set up shells
RPI = "pi@10.8.244.74"
RPI_PASSWORD = "raspberry"

SQUIRREL = "squirrel@squirrel"
SQUIRREL_PASSWORD = "asdf"

SSH = ["ssh", "-t", "-t", SQUIRREL]
COMMAND = "printenv"

ssh = subprocess.Popen(SSH, 
                       stdin=subprocess.PIPE, 
#                       stdout=subprocess.PIPE, 
                       stderr=subprocess.PIPE)

ssh.stdin.write(SQUIRREL_PASSWORD + "\n")
ssh.stdin.flush()
ssh.stdin.write(COMMAND + "\n")

print "holding loop"
while True:
    time.sleep(1)

