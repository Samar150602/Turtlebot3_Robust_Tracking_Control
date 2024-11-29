#!/usr/bin/env python3

# 8 shape without controller
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as transformations
import math
import matplotlib.pyplot as plt
import os
import csv

# Initializing lists to store the x and y positions of the robot from Odom topic
x_positions = []
y_positions = []

# Initialize desired positions
x_des = []
y_des = []

# Save robot data from movement of the turtlebot in simulation and/or Hardware
once = True
file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/4-RTIC/2-Project/3-ROS/TrajectoryFollowing_ws/src/trajectory_control_pkg/robot_data_csv_files/8shape_data.csv'

if os.path.exists(file_path):
    os.remove(file_path)  # Delete the file

def odom_callback(data):
    global theta
    
    # Extracting the current position from the odometry message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    quaternion = data.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    _, _, theta = transformations.euler_from_quaternion(quat)
    
    # Append the current position to the lists
    x_positions.append(x)
    y_positions.append(y)

def figure_eight_trajectory(A, B, omega):
    global once
    rospy.init_node('turtlebot3_eight', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Publish to /cmd_vel
    rospy.Subscriber('/odom', Odometry, odom_callback)  # Subscribe to odometry data
    rate = rospy.Rate(30)  # 30Hz

    start_time = rospy.Time.now().to_sec()  # Start time of the Simulation
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time  # Calculate the elapsed time
        
        # Desired trajectory
        x_target = A * math.sin(omega * t)
        y_target = B * math.sin(2 * omega * t)

        # First derivative (velocity)
        x_target_dot = A * omega * math.cos(omega * t)
        y_target_dot = 2 * B * omega * math.cos(2 * omega * t)

        # Append desired positions for plotting
        x_des.append(x_target)
        y_des.append(y_target)

        # Current position of the robot
        if x_positions and y_positions:  # Ensure there's data available
            x_current = x_positions[-1]
            y_current = y_positions[-1]
            
            # Desired velocities
            linear_velocity = math.sqrt(x_target_dot**2 + y_target_dot**2)
            theta_target = math.atan2(y_target_dot, x_target_dot)
            angular_velocity = 2.0 * (theta_target - theta)  # Proportional control

            # Prepare velocity message
            velocity_msg = Twist()
            velocity_msg.linear.x = linear_velocity
            velocity_msg.angular.z = angular_velocity

            # Write data to a CSV file
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)

                # Write header if needed
                if once:
                    writer.writerow(["x", "y", "theta", "theta_degree", "v", "omega"])
                    once = False

                # Convert to degrees
                theta_degrees = math.degrees(theta)

                data = [
                    x_current,
                    y_current,
                    theta,
                    theta_degrees,
                    linear_velocity,
                    angular_velocity
                ]

                # Write data to the file
                writer.writerow(data)
                print("Data written:", data)

                # Flush the buffer to ensure data is written to the disk
                file.flush()

            # Publish the velocity
            pub.publish(velocity_msg)
        rate.sleep()

def plot_trajectory():
    # Plot the 8-shaped trajectory
    plt.figure()
    plt.plot(x_positions, y_positions, label="Robot Trajectory")
    plt.plot(x_des, y_des, label="Desired Trajectory")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.title("8-Shaped Trajectory without controller")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == '__main__':
    try:
        figure_eight_trajectory(A=1, B=-1, omega=0.04) 
    except rospy.ROSInterruptException:
        pass
    finally:
        plot_trajectory()  # Plot the trajectory when the program stops
