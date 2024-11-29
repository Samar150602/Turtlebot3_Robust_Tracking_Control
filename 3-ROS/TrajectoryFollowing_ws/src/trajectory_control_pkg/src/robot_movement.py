#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('robot_movement', anonymous=True)                   # initializing rosnode
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)            # create a publisher for the '/cmd_vel'
    
rate = rospy.Rate(10)  # 10 Hz (adjustable rate)

twist = Twist() # creating an instance of twist message

twist.linear.x = 0.125
twist.angular.z = 0.0    

while not rospy.is_shutdown():
    rospy.loginfo("Moving Forward")
    
    pub.publish(twist)
    
    rate.sleep()
                           




