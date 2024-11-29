#!/usr/bin/env python3

# Circular Trajectory 
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
file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/4-RTIC/2-Project/3-ROS/TrajectoryFollowing_ws/src/trajectory_control_pkg/robot_data_csv_files/circle_data.csv'

if os.path.exists(file_path):
    os.remove(file_path) # Delete the file

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


def circular_trajectory(radius, angular_velocity):
    global once
    rospy.init_node('turtlebot3_circle', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publish to /cmd_vel
    rospy.Subscriber('/odom', Odometry, odom_callback)  # Subscribe to odometry data
    rate = rospy.Rate(30)  # 30Hz

    start_time = rospy.Time.now().to_sec() # Start time of the Simulation
    
    while not rospy.is_shutdown():
        
        t = rospy.Time.now().to_sec() - start_time # Calculate the elapsed time
        
        x_des_pos =  radius * math.cos(angular_velocity * t)
        y_des_pos =  radius * math.sin(angular_velocity * t)

        # Append desired positions for plotting
        x_des.append(x_des_pos)
        y_des.append(y_des_pos)

        # Parametric circle equations:
        vx = -radius * angular_velocity * math.sin(angular_velocity * t)
        vy = radius * angular_velocity * math.cos(angular_velocity * t)

        # Current position of the robot
        if x_positions and y_positions:  # Ensure there's data available
            x_current = x_positions[-1]
            y_current = y_positions[-1]
        
            linear_velocity = math.sqrt(vx**2 + vy**2)

            # Compute linear velocity and angular velocity
            velocity_msg = Twist()
            velocity_msg.linear.x = linear_velocity
            velocity_msg.angular.z = angular_velocity
            
            # write data to an excel file:
            with open(file_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    
                # Write header if needed
                    if once == True:
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
    # Plot the circles    
    plt.figure()
    plt.plot(x_positions, y_positions, label="Robot Trajectory")
    # plt.plot(x_des, y_des, label="Desired Trajectory")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.title("Circular Trajectory without controller")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == '__main__':
    try:
        circular_trajectory(radius=1.0, angular_velocity=0.5) # Draw circle
    except rospy.ROSInterruptException:
        pass
    finally:
        plot_trajectory() # Plot the trajectory (when the program is stopped)


