import paramiko

def interExecute(host,port,username,password,cmd):
        """Execute the given commands in an interactive shell."""
        transport = paramiko.Transport((host, port))            
        transport.connect(username = username, password = password)
        chan = paramiko.transport.open_session()
        chan.setblocking(0)
        chan.invoke_shell()

        out = ''

        chan.send(cmd+'\n')

        tCheck = 0

        # Wait for it.....
        while not chan.recv_ready():
            time.sleep(10)
            tCheck+=1
            if tCheck >= 6:
                print 'time out'#TODO: add exeption here 
                return False
        out = chan.recv(1024)
 
        return out

interExecute(host="10.9.160.238", 
             port=22,
             username="squirrel",
             password="asdf",
             cmd="printenv")
