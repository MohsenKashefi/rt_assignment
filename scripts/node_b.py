#!/usr/bin/env python3

import rospy
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Input, InputResponse

# Initialize node and log info
rospy.init_node('last_target_service')
rospy.loginfo("Last target node initialized")

# Initialize last desired x and y positions
last_desired_x = 0
last_desired_y = 0

# Service callback function
def handle_input_request(request):
    # Retrieve the last desired x and y positions from ROS parameters
    last_desired_x = rospy.get_param('/des_pos_x')
    last_desired_y = rospy.get_param('/des_pos_y')

    # Create and return the response message
    response = InputResponse()
    response.input_x = last_desired_x
    response.input_y = last_desired_y
    return response

# Function to provide the service
def provide_input_service():
    # Provide a service named 'input', using the custom service type Input
    rospy.Service('input', Input, handle_input_request)
    rospy.spin()  # Keep the node running

# Main function
if __name__ == "__main__":
    # Start providing the input service
    provide_input_service()

