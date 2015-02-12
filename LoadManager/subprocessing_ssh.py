import subprocess
import sys

# Ports are handled in ~/.ssh/config since we use OpenSSH
USER = "pi" 
HOST = "10.8.244.74"

COMMAND = "python test_stdout.py"
 
ssh = subprocess.Popen(["ssh", "%s@%s" % USER, HOST, COMMAND],
                       shell=False,
                       stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE)
result = ssh.stdout.readlines()
if result == []:
    error = ssh.stderr.readlines()
    print >>sys.stderr, "ERROR: %s" % error
else:
    print result
