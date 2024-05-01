#!/usr/bin/env python3

"""
.. module:: rt_assignment2
   :platform: Unix
   :synopsis: Python module for the node_a
.. moduleauthor:: Mohsen Kashefi

This code implements a the node_a.

Subscriber:
    - /node_a/pose: Subscribes to the pose.
"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus
import threading

def publish_position_and_velocity(msg, pos_vel_publisher):
    """
    Publishes the current position and velocity information.

    :param msg: Odometry message containing position and velocity information.
    :type msg: nav_msgs.msg.Odometry
    :param pos_vel_publisher: Publisher for position and velocity information.
    :type pos_vel_publisher: rospy.Publisher
    """
    current_pos = msg.pose.pose.position
    current_vel_linear = msg.twist.twist.linear
    current_vel_angular = msg.twist.twist.angular

    pos_and_vel = Vel()
    pos_and_vel.pos_x = current_pos.x
    pos_and_vel.pos_y = current_pos.y
    pos_and_vel.vel_x = current_vel_linear.x
    pos_and_vel.vel_z = current_vel_angular.z

    pos_vel_publisher.publish(pos_and_vel)

def get_user_input(prompt):
    """
    Retrieves user input with a prompt.

    :param prompt: Prompt to display to the user.
    :type prompt: str
    :return: User input
    :rtype: str
    """
    user_input = None
    input_lock = threading.Lock()

    def input_thread():
        nonlocal user_input
        with input_lock:
            user_input = input(prompt)

    input_handler = threading.Thread(target=input_thread)
    input_handler.start()
    input_handler.join(timeout=1)  # Timeout to avoid blocking forever

    with input_lock:
        return user_input

def get_new_goal():
    """
    Retrieves a new goal from user input.

    :return: New goal and its coordinates
    :rtype: tuple(assignment_2_2023.msg.PlanningGoal, tuple(float, float))
    """
    try:
        input_x = float(input("x: "))
        input_y = float(input("y: "))
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a valid number.")
        return None, None

    rospy.set_param('/des_pos_x', input_x)
    rospy.set_param('/des_pos_y', input_y)

    goal = PlanningGoal()
    goal.target_pose.pose.position.x = input_x
    goal.target_pose.pose.position.y = input_y

    return goal, (input_x, input_y)

def handle_goal_commands():
    """
    Handles goal commands from the user.
    """
    rospy.init_node('set_target_client')

    pos_vel_publisher = rospy.Publisher("/pos_vel", Vel, queue_size=1)
    action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    action_client.wait_for_server()

    goal_cancelled = True

    # Setup subscriber outside the loop
    rospy.Subscriber("/odom", Odometry, lambda msg: publish_position_and_velocity(msg, pos_vel_publisher))

    while not rospy.is_shutdown():
        command = get_user_input("If you want to define a new goal, press 'd'. If you want to cancel the goal, press 'q': ")

        target_pos_x = rospy.get_param('/des_pos_x')
        target_pos_y = rospy.get_param('/des_pos_y')

        current_goal = PlanningGoal()
        current_goal.target_pose.pose.position.x = target_pos_x
        current_goal.target_pose.pose.position.y = target_pos_y
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)

        if command == 'd':
            new_goal, new_goal_coords = get_new_goal()
            if new_goal:
                action_client.send_goal(new_goal)
                goal_cancelled = False
        elif command == 'q':
            if not goal_cancelled:
                goal_cancelled = True
                action_client.cancel_goal()
                rospy.loginfo("Current goal has been cancelled")
            else:
                rospy.loginfo("No active goal to cancel")
        else:
            rospy.logwarn("This is an incorrect command. Please choose between 'd' and 'q'.")

        rospy.loginfo("Last received goal: target_x = %f, target_y = %f", current_goal.target_pose.pose.position.x, current_goal.target_pose.pose.position.y)
        rospy.sleep(1)  # To prevent the loop from consuming too much CPU time

def main():
    """
    Main function to run the goal handler.
    """
    handle_goal_commands()

if __name__ == '__main__':
    main()

# !/usr/bin/env python3

# import rospy
# import threading
# from std_msgs.msg import String
# from ipywidgets import widgets, interact

# # Define ROS node and publisher
# rospy.init_node('user_interface')  # Initialize only once
# publisher = rospy.Publisher('/user_commands', String, queue_size=10)

# # Define callback function for sending commands
# def send_command(command):
#     publisher.publish(command)

# # Define UI elements
# command_dropdown = widgets.Dropdown(
#     options=['Define New Goal (d)', 'Cancel Goal (q)'],
#     description='Command:'
# )

# x_input = widgets.FloatText(
#     description='X:'
# )

# y_input = widgets.FloatText(
#     description='Y:'
# )

# submit_button = widgets.Button(
#     description='Submit'
# )

# # Define callback function for submit button click
# def on_submit_clicked(b):
#     command = command_dropdown.value[0]
#     if command == 'd':
#         x = x_input.value
#         y = y_input.value
#         send_command(f'd {x} {y}')
#     elif command == 'q':
#         send_command('q')

# submit_button.on_click(on_submit_clicked)

# # Display UI
# display(command_dropdown, x_input, y_input, submit_button)

# # ROS subscriber
# def handle_user_commands(data):
#     rospy.loginfo(f"Received command: {data.data}")

# subscriber = rospy.Subscriber('/user_commands', String, handle_user_commands)

# # Spin ROS
# rospy.spin()
