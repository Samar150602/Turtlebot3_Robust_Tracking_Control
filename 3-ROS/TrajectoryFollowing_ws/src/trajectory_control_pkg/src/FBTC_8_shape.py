#!/usr/bin/env python3

# Differential Flatness Based Tracking Control

import os
import csv
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as transformations
import math
import matplotlib.pyplot as plt
import pandas as pd  # Import pandas for saving data to Excel

# Initialize global lists to store the x and y positions
x_positions = []
y_positions = []
yaw_angles = []  # List to store yaw angles (orientation)


# Initialize desired positions
x_des = []
y_des = []

# Characteristic equation of Denominator Polynomial of Transfer Function equation = (s)^2 + 2(zeta)(omega)(s) + (omega)^2
# FBTC Constants - omega and zeta = 0.1
Kp_x = 0.01
Kd_x = 0.24
Kp_y = 0.01
Kd_y = 0.24

# Initialize previous errors and time
previous_error_x = 0
previous_error_y = 0
previous_t = 0

theta = 0

once = True
file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/4-RTIC/2-Project/3-ROS/TrajectoryFollowing_ws/src/trajectory_control_pkg/robot_data_csv_files/FBTC_8shape_data.csv'

if os.path.exists(file_path):
    # Delete the file
    os.remove(file_path)
        
def odom_callback(data):
    global theta
    
    # Extract the current position from the odometry message
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
    global previous_error_x, previous_error_y, previous_t, once
    
    rospy.init_node('turtlebot3_figure_eight', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)  # Subscribe to odometry data
    rate = rospy.Rate(30)  # Control frequency in Hz

    start_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        
        if t > 157:
            rospy.signal_shutdown("Shutdown initiated by user.")
            break

        # # Desired trajectory
        x_target = A * math.sin(omega * t)
        y_target = B * math.sin(2 * omega * t)
        
        # First derivative (velocity)
        x_target_dot = A * omega * math.cos(omega * t)
        y_target_dot = 2 * B * omega * math.cos(2* omega * t)

        # Second derivative (acceleration)
        x_target_double_dot = -A * (omega**2) * math.sin(omega * t)
        y_target_double_dot = -4 * B * (omega**2) * math.sin(2* omega * t)
        
        # Append desired positions for plotting
        x_des.append(x_target)
        y_des.append(y_target)
        
        # Current position of the robot
        if x_positions and y_positions:  # Ensure there's data available
            x_current = x_positions[-1]
            y_current = y_positions[-1]
            
            # Calculate position errors
            error_x = x_target - x_current
            error_y = y_target - y_current
            
            # Calculate derivative of errors
            if previous_t > 0:
                dt = t - previous_t
                derivative_error_x = (error_x - previous_error_x) / dt
                derivative_error_y = (error_y - previous_error_y) / dt
            else:
                derivative_error_x = 0
                derivative_error_y = 0

            # Update previous error and time
            previous_error_x = error_x
            previous_error_y = error_y
            previous_t = t
            
            # PID control for v1 and v2
            v1 = x_target_double_dot + (Kp_x * error_x) + (Kd_x * derivative_error_x)
            v2 = y_target_double_dot + (Kp_y * error_y) + (Kd_y * derivative_error_y)
            
            # Calculate control inputs u1 (linear velocity) and u2_omega (angular velocity)
            linear_vel = math.sqrt(x_target_dot**2 + y_target_dot**2) 
            angular_vel = ((x_target_dot * v2) - (y_target_dot * v1)) / (x_target_dot**2 + y_target_dot**2)

            # Optional: Saturate velocities
            max_linear_speed = 0.3  # Set maximum linear speed
            max_angular_speed = 0.8  # Set maximum angular speed

            linear_vel = max(min(linear_vel, max_linear_speed), -max_linear_speed)
            angular_vel = max(min(angular_vel, max_angular_speed), -max_angular_speed)
            
            # Create Twist message
            velocity_msg = Twist()
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            
            with open(file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                
            # Write header if needed
                if once == True:
                    writer.writerow(["time", "x", "y", "theta", "theta_degree", "v", "omega"])
                    once = False
                
                # Convert to degrees
                theta_degrees = math.degrees(theta)

                data = [
                    t,
                    x_current,
                    y_current,
                    theta,
                    theta_degrees,
                    linear_vel,
                    angular_vel
                ]
                
                # Write data to the file
                writer.writerow(data)
                print("Data written:", data)
                
                # Flush the buffer to ensure data is written to the disk
                file.flush()
            
            # Publish the command
            pub.publish(velocity_msg)
            
        # Sleep to maintain the loop rate
        rate.sleep()

def plot_trajectory():
    # Plot the collected x and y positions
    plt.figure()
    plt.plot(x_positions, y_positions, label="Robot Trajectory")
    plt.plot(x_des, y_des, label="Desired Trajectory")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.title("8-Shape Trajectory Tracking with FBTC")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == '__main__':
    try:
        # Set parameters for the figure-eight trajectory
        figure_eight_trajectory(A=1, B=-1, omega=0.04) # A=1, B=1, omega=0.1065
    except rospy.ROSInterruptException:
        pass
    finally:
        # Plot the trajectory after the loop exits (when the program is stopped)
        plot_trajectory()

