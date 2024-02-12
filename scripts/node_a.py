#!/usr/bin/env python3

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
    handle_goal_commands()

if __name__ == '__main__':
    main()

