"""
Launch file to start all processes for thesis demo.
"""

import paramiko
import time, sys, os


# set up shells
squirrel = paramiko.SSHClient()
squirrel.set_missing_host_key_policy(paramiko.AutoAddPolicy())
squirrel.connect("10.9.160.238", 
                 username="squirrel", 
                 password="asdf")

#asdf = paramiko.SSHClient()
#asdf.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#asdf.connect("10.8.190.94", 
#             username="asdf", 
#             password="asdf")

stdin, stdout, stderr = squirrel.exec_command("source .junk; printenv")
print stdout.read()
print stderr.read()

print "Going into a holding loop...\n"
while True:
    time.sleep(1)
