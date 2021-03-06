#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist

turn = 1.0

def run():
    
    # publish twist messages to /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, 10)
    rospy.init_node('drive_in_circles')

    global turn
    twist = Twist()

    while not rospy.is_shutdown():
        rospy.loginfo("Turning...")
        twist.linear.x = 0.2; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
    
        pub.publish(twist)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
