#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

# Desired heading (in degrees)
desired_heading = -64.64  # You can change this to any angle you want

# Set up the publisher for velocity commands
cmd_vel_pub = None

def odom_callback(data):
    global cmd_vel_pub
    
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
    
    # Compute the error in heading (desired heading - current heading)
    heading_error = desired_heading - yaw_degrees
    heading_error = (heading_error + 180) % 360 - 180  # Normalize error to [-180, 180]
    
    # Create a Twist message to send velocity commands
    cmd = Twist()

    # If heading_error is small, stop the robot, else turn towards the desired heading
    if abs(heading_error) > 1:  # You can adjust the tolerance here
        # Proportional control for angular velocity (simplified)
        cmd.angular.z = 0.1 * heading_error  # You can adjust the gain (0.1) as needed
    else:
        # If close enough to the desired heading, stop rotating
        cmd.angular.z = 0.0
    
    # Publish the velocity command
    cmd_vel_pub.publish(cmd)

    # Log current position and heading
    rospy.loginfo(f"Position -> X: {x:.2f}, Y: {y:.2f}, Heading (Yaw): {yaw_degrees:.2f} degrees, Error: {heading_error:.2f} degrees")

def main():
    global cmd_vel_pub
    
    # Initialize the ROS node
    rospy.init_node('move_to_heading', anonymous=True)
    
    # Set up a publisher to send velocity commands to the robot
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Subscribe to the Odometry topic to get the robot's position and orientation
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Spin to keep the node running and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
