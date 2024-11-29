#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(data):
    # Extract position (x, y)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    # Extract orientation in quaternion (x, y, z, w)
    orientation_q = data.pose.pose.orientation
    quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    
    # Convert quaternion to Euler angles
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    # Convert yaw (radians) to degrees for easier understanding
    yaw_degrees = yaw * (180.0 / 3.141592653589793)
    
    # Print x, y, and heading (yaw)
    rospy.loginfo(f"Position -> X: {x:.2f}, Y: {y:.2f}, Heading (Yaw): {yaw_degrees:.2f} degrees")

def main():
    rospy.init_node('print_turtlebot_position_heading', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
