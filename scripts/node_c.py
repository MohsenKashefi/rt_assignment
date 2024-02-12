#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

# Initialize global variables for average velocity and distance
average_vel_x = 0
distance = 0

def get_distance_and_average_velocity(msg):
    global distance
    global average_vel_x

    # Get desired x and y positions from the parameter server
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')

    # Get the window size for the velocity calculation from the parameter server
    velocity_window_size = rospy.get_param('/window_size')

    # Get actual x and y positions from the message
    actual_x = msg.pos_x
    actual_y = msg.pos_y

    # Calculate the distance between the desired and actual positions
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)

    # Calculate the average velocity
    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

def get_values(_):
    global distance
    global average_vel_x

    # Log information about the distance and average velocity
    rospy.loginfo(f"Distance: {distance}, Average Velocity X: {average_vel_x}")

    # Return a response with the distance and average velocity
    return Ave_pos_velResponse(distance, average_vel_x)

def main():
    # Initialize the node with the name 'info_service'
    rospy.init_node('info_service')
    rospy.loginfo("Info service node initialized")

    # Subscribe to the '/pos_vel' topic, using the custom message type Vel
    rospy.Subscriber("/pos_vel", Vel, get_distance_and_average_velocity)

    # Provide a service named 'info_service', using the custom service type Ave_pos_vel
    rospy.Service("info_service", Ave_pos_vel, get_values)

    # Set the rate at which the loop will run (in Hz)
    rate = rospy.Rate(10)  # e.g., 10 Hz

    while not rospy.is_shutdown():
        # Log desired information
        rospy.loginfo("Logging desired information...")
        rospy.loginfo(f"Current Distance: {distance}, Current Average Velocity X: {average_vel_x}")

        # Your additional logic goes here (if needed)
        rate.sleep()

# Main function
if __name__ == "__main__":
    main()

