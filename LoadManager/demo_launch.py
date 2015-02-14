"""
Launch file to start all processes for thesis demo.
"""

from collections import deque
import spur
import time, sys, os

start = time.time()

# set up shells
squirrel = spur.SshShell(hostname="10.9.160.238", 
                         username="squirrel", 
                         password="asdf")
asdf = spur.SshShell(hostname="10.8.190.94", 
                     username="asdf", 
                     password="asdf")
 
# set up environments
squirrelEnv = {TERM=xterm-256color
SHELL=/bin/bash
ROS_ROOT=/opt/ros/hydro/share/ros
XDG_SESSION_COOKIE=59f8830617570f2292e181190000000d-1423869004.482697-819750221
SSH_CLIENT=10.8.72.243 52820 22
ROS_PACKAGE_PATH=/opt/ros/hydro/share:/opt/ros/hydro/stacks:/home/squirrel/thesis/ROS/helloworld/turtlebot_driver_test/
ROS_MASTER_URI=http://squirrel-Precision-M6300:11311
SSH_TTY=/dev/pts/2
USER=squirrel
LD_LIBRARY_PATH=/opt/ros/hydro/lib:/opt/ros/hydro/lib/python2.7/dist-packages
LS_COLORS=rs=0:di=01;34:ln=01;36:mh=00:pi=40;33:so=01;35:do=01;35:bd=40;33;01:cd=40;33;01:or=40;31;01:su=37;41:sg=30;43:ca=30;41:tw=30;42:ow=34;42:st=37;44:ex=01;32:*.tar=01;31:*.tgz=01;31:*.arj=01;31:*.taz=01;31:*.lzh=01;31:*.lzma=01;31:*.tlz=01;31:*.txz=01;31:*.zip=01;31:*.z=01;31:*.Z=01;31:*.dz=01;31:*.gz=01;31:*.lz=01;31:*.xz=01;31:*.bz2=01;31:*.bz=01;31:*.tbz=01;31:*.tbz2=01;31:*.tz=01;31:*.deb=01;31:*.rpm=01;31:*.jar=01;31:*.war=01;31:*.ear=01;31:*.sar=01;31:*.rar=01;31:*.ace=01;31:*.zoo=01;31:*.cpio=01;31:*.7z=01;31:*.rz=01;31:*.jpg=01;35:*.jpeg=01;35:*.gif=01;35:*.bmp=01;35:*.pbm=01;35:*.pgm=01;35:*.ppm=01;35:*.tga=01;35:*.xbm=01;35:*.xpm=01;35:*.tif=01;35:*.tiff=01;35:*.png=01;35:*.svg=01;35:*.svgz=01;35:*.mng=01;35:*.pcx=01;35:*.mov=01;35:*.mpg=01;35:*.mpeg=01;35:*.m2v=01;35:*.mkv=01;35:*.webm=01;35:*.ogm=01;35:*.mp4=01;35:*.m4v=01;35:*.mp4v=01;35:*.vob=01;35:*.qt=01;35:*.nuv=01;35:*.wmv=01;35:*.asf=01;35:*.rm=01;35:*.rmvb=01;35:*.flc=01;35:*.avi=01;35:*.fli=01;35:*.flv=01;35:*.gl=01;35:*.dl=01;35:*.xcf=01;35:*.xwd=01;35:*.yuv=01;35:*.cgm=01;35:*.emf=01;35:*.axv=01;35:*.anx=01;35:*.ogv=01;35:*.ogx=01;35:*.aac=00;36:*.au=00;36:*.flac=00;36:*.mid=00;36:*.midi=00;36:*.mka=00;36:*.mp3=00;36:*.mpc=00;36:*.ogg=00;36:*.ra=00;36:*.wav=00;36:*.axa=00;36:*.oga=00;36:*.spx=00;36:*.xspf=00;36:
CPATH=/opt/ros/hydro/include
MAIL=/var/mail/squirrel
PATH=/opt/ros/hydro/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
PWD=/home/squirrel
ROS_HOSTNAME=squirrel-Precision-M6300
LANG=en_US.UTF-8
ROSLISP_PACKAGE_DIRECTORIES=
SHLVL=1
HOME=/home/squirrel
ROS_DISTRO=hydro
PYTHONPATH=/opt/ros/hydro/lib/python2.7/dist-packages
LOGNAME=squirrel
ROS_IP=10.9.160.238
SSH_CONNECTION=10.8.72.243 52820 10.9.160.238 22
PKG_CONFIG_PATH=/opt/ros/hydro/lib/pkgconfig
LESSOPEN=| /usr/bin/lesspipe %s
CMAKE_PREFIX_PATH=/opt/ros/hydro
LESSCLOSE=/usr/bin/lesspipe %s %s
ROS_ETC_DIR=/opt/ros/hydro/etc/ros}
asdfEnv = {"SSH_AGENT_PID" : "1497",
           "GPG_AGENT_INFO" : "/tmp/keyring-RyYGKl/gpg:0:1",
           "SHELL" : "/bin/bash",
           "TERM" : "xterm",
           "ROS_ROOT" : "/opt/ros/hydro/share/ros",
           "XDG_SESSION_COOKIE" : "0be3f6bebcf14c7b4aa62a7a0000000f-1423775401.555421-1408391649",
           "ROS_PACKAGE_PATH" : "/opt/ros/hydro/share:/opt/ros/hydro/stacks",
           "ROS_MASTER_URI" : "http://squirrel:11311",
           "WINDOWID" : "62914565",
           "GNOME_KEYRING_CONTROL" : "/tmp/keyring-RyYGKl",
           "USER" : "asdf",
           "LD_LIBRARY_PATH" : "/opt/ros/hydro/lib:/opt/ros/hydro/lib/python2.7/dist-packages",
           "LS_COLORS" : "rs=0:di=01;34:ln=01;36:mh=00:pi=40;33:so=01;35:do=01;35:bd=40;33;01:cd=40;33;01:or=40;31;01:su=37;41:sg=30;43:ca=30;41:tw=30;42:ow=34;42:st=37;44:ex=01;32:*.tar=01;31:*.tgz=01;31:*.arj=01;31:*.taz=01;31:*.lzh=01;31:*.lzma=01;31:*.tlz=01;31:*.txz=01;31:*.zip=01;31:*.z=01;31:*.Z=01;31:*.dz=01;31:*.gz=01;31:*.lz=01;31:*.xz=01;31:*.bz2=01;31:*.bz=01;31:*.tbz=01;31:*.tbz2=01;31:*.tz=01;31:*.deb=01;31:*.rpm=01;31:*.jar=01;31:*.war=01;31:*.ear=01;31:*.sar=01;31:*.rar=01;31:*.ace=01;31:*.zoo=01;31:*.cpio=01;31:*.7z=01;31:*.rz=01;31:*.jpg=01;35:*.jpeg=01;35:*.gif=01;35:*.bmp=01;35:*.pbm=01;35:*.pgm=01;35:*.ppm=01;35:*.tga=01;35:*.xbm=01;35:*.xpm=01;35:*.tif=01;35:*.tiff=01;35:*.png=01;35:*.svg=01;35:*.svgz=01;35:*.mng=01;35:*.pcx=01;35:*.mov=01;35:*.mpg=01;35:*.mpeg=01;35:*.m2v=01;35:*.mkv=01;35:*.webm=01;35:*.ogm=01;35:*.mp4=01;35:*.m4v=01;35:*.mp4v=01;35:*.vob=01;35:*.qt=01;35:*.nuv=01;35:*.wmv=01;35:*.asf=01;35:*.rm=01;35:*.rmvb=01;35:*.flc=01;35:*.avi=01;35:*.fli=01;35:*.flv=01;35:*.gl=01;35:*.dl=01;35:*.xcf=01;35:*.xwd=01;35:*.yuv=01;35:*.cgm=01;35:*.emf=01;35:*.axv=01;35:*.anx=01;35:*.ogv=01;35:*.ogx=01;35:*.aac=00;36:*.au=00;36:*.flac=00;36:*.mid=00;36:*.midi=00;36:*.mka=00;36:*.mp3=00;36:*.mpc=00;36:*.ogg=00;36:*.ra=00;36:*.wav=00;36:*.axa=00;36:*.oga=00;36:*.spx=00;36:*.xspf=00;36:",
           "XDG_SESSION_PATH" : "/org/freedesktop/DisplayManager/Session0",
           "XDG_SEAT_PATH" : "/org/freedesktop/DisplayManager/Seat0",
           "CPATH" : "/opt/ros/hydro/include",
           "SSH_AUTH_SOCK" : "/tmp/keyring-RyYGKl/ssh",
           "DEFAULTS_PATH" : "/usr/share/gconf/ubuntu-2d.default.path",
           "SESSION_MANAGER" : "local/asdf:@/tmp/.ICE-unix/1461,unix/asdf:/tmp/.ICE-unix/1461",
           "XDG_CONFIG_DIRS" : "/etc/xdg/xdg-ubuntu-2d:/etc/xdg",
           "PATH" : "/opt/ros/hydro/bin:/usr/lib/lightdm/lightdm:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games",
           "DESKTOP_SESSION" : "ubuntu-2d",
           "PWD" : "/home/asdf",
           "ROS_HOSTNAME" : "asdf",
           "LANG" : "en_US.UTF-8",
           "MANDATORY_PATH" : "/usr/share/gconf/ubuntu-2d.mandatory.path",
           "UBUNTU_MENUPROXY" : "libappmenu.so",
           "GDMSESSION" : "ubuntu-2d",
           "SHLVL" : "1",
           "HOME" : "/home/asdf",
           "ROS_DISTRO" : "hydro",
           "GNOME_DESKTOP_SESSION_ID" : "this-is-deprecated",
           "PYTHONPATH" : "/opt/ros/hydro/lib/python2.7/dist-packages",
           "LOGNAME" : "asdf",
           "ROS_IP" : "10.8.190.94",
           "XDG_DATA_DIRS" : "/usr/share/ubuntu-2d:/usr/share/gnome:/usr/local/share/:/usr/share/",
           "DBUS_SESSION_BUS_ADDRESS" : "unix:abstract=/tmp/dbus-xsMO9sL6ZE,guid=0fb59a3d883024602f2b394a0000001e",
           "PKG_CONFIG_PATH" : "/opt/ros/hydro/lib/pkgconfig",
           "LESSOPEN" : "| /usr/bin/lesspipe %s",
           "CMAKE_PREFIX_PATH" : "/opt/ros/hydro",
           "DISPLAY" : ":0",
           "XDG_CURRENT_DESKTOP" : "Unity",
           "LESSCLOSE" : "/usr/bin/lesspipe %s %s",
           "ROS_ETC_DIR" : "/opt/ros/hydro/etc/ros",
           "XAUTHORITY" : "/home/asdf/.Xauthority",
           "COLORTERM" : "gnome-terminal"}

# set up a queue of commands to execute and processes launched
commandQueue = deque()
commandQueue.append({"command" : ["roscore"],
                     "host"    : asdf})


commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_bringup", 
                                  "minimal.launch"],
                     "host"    : asdf})

"""
commandQueue.append({"command" : ["roslaunch", 
                                  "turtlebot_navigation", 
                                  "gmapping_demo.launch"],
                     "host"    : asdf})
commandQueue.append({"command" : ["roslaunch",
                                  "turtlebot_rviz_launchers",
                                  "view_navigation.launch"],
                     "host"    : asdf})
"""

processQueue = deque()

try:

    # launch processes in order
    for task in commandQueue:
        command = task["command"]
        shell = task["host"]
        
        process = shell.spawn(command,
                              update_env=asdfEnv,
                              stdout=sys.stdout, 
                              store_pid=True)
        
        processQueue.append({"process" : process,
                             "command" : command})
        print str(command) + " pid: " + str(process.pid)
        

        # wait until each command is done
        time.sleep(10) # for now, just wait 10 seconds


    # wait for keyboard interrupt, and then kill all processes
    while True:
        time.sleep(1)
        
except KeyboardInterrupt:
    while len(processQueue) > 0
        task = processQueue.pop()
        print "Terminating " + str(task["command"])
        task["process"].send_signal(15) # kill -15 is a SIGTERM signal

    print "All processes shut down cleanly."
    sys.exit()
