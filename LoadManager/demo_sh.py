"""
Launch file to start all processes for thesis demo.
Uses sh library to launch commands on the local shell.

This file launches ssh interactive login shells on each host and 
sets up processes on each of them by sending commands over stdin.
"""

from sh import ssh
import os, sys

# set up shells
RPI = "pi@10.8.244.74"
RPI_PASSWORD = "raspberry"

SQUIRREL = "squirrel@squirrel"
SQUIRREL_PASSWORD = "asdf"

COMMAND = "printenv"

# open stdout in unbuffered mode
sys.stdout = os.fdopen(sys.stdout.fileno(), "wb", 0)

aggregated = ""
def ssh_interact(char, stdin):
    global aggregated
    sys.stdout.write(char.encode())
    aggregated += char
    if aggregated.endswith("password: "):
        stdin.put(RPI_PASSWORD + "\n")

p = ssh("-t -t " + RPI, _out=ssh_interact, _out_bufsize=0, _tty_in=True)
#p.printenv()
p.wait()


