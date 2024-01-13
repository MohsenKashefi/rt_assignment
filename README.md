# ROS Navigation System

This repository contains ROS (Robot Operating System) nodes and services for navigation and information retrieval.

## Contents

1. [Overview](#overview)
2. [Key Features](#Key Features)
3. [Dependencies](#dependencies)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Pseudocode](#Pseudocode)


## Overview

The repository includes three main components:

1. **node_a**: Allows users to set and cancel navigation goals for a robot.
2. **node_b**: Provides a service to retrieve information about the last desired x and y positions.
3. **node_c**: Calculates and provides information about the distance and average velocity.

## Key Features

### 1. node_a

- **Goal Setting:** Users can interactively set new navigation goals by specifying target coordinates, triggering the robot's navigation planning.
  
- **Goal Cancellation:** Users have the ability to cancel an ongoing navigation goal, providing flexibility in robot control.

- **Integration with Actionlib:** The script seamlessly integrates with ROS actionlib, allowing for efficient, goal-driven behavior.

### 2. node_b

- **Information Retrieval:** This service allows users to query and obtain information about the last set desired x and y positions, offering insights into recent navigation goals.

- **Parameter-based Storage:** The last desired positions are stored in ROS parameters for easy retrieval.

### 3. node_c

- **Distance Calculation:** The script calculates the distance between the desired and actual robot positions, providing valuable information about navigation progress.

- **Average Velocity:** Information on the average velocity, computed over a specified window size, offers insights into the robot's movement dynamics.


## Dependencies

- ROS (Melodic or later)
- Python 3
- Required ROS packages (if any, specify them here)

## Installation

1. Clone this repository:

    ```bash
    git clone https://github.com/MohsenKashefi/rt_second_assignment.git
    ```

# set_target_client.py

# Initialize ROS node
initialize ROS node

# Initialize publishers, subscribers, and action client
initialize Pose publisher, Twist publisher, Odometry subscriber, and Planning action client

# Main loop
while not ROS shutdown:
    # Prompt user for input
    prompt user for input

    # Check user input
    if input is 'd':
        # Get new goal coordinates from user
        new_goal_coordinates = get_user_input("Enter new goal coordinates (x, y): ")
        
        # Create a new PlanningGoal message
        new_goal = create_planning_goal(new_goal_coordinates)

        # Send new goal to action server
        send_goal_to_action_server(new_goal)

    elif input is 'q':
        # Cancel current goal if exists
        cancel_current_goal()

# Function to get new goal coordinates from the user
function get_user_input(prompt):
    display prompt to user
    wait for user input
    return user_input

# Function to create a new PlanningGoal message
function create_planning_goal(coordinates):
    create a new PlanningGoal message
    set the target pose with provided coordinates
    return the PlanningGoal message

# Function to send a new goal to the action server
function send_goal_to_action_server(goal):
    send the provided goal to the Planning action server

# Function to cancel the current goal
function cancel_current_goal():
    check if there is an active goal
    if active goal exists:
        cancel the active goal
        log information about goal cancellation


