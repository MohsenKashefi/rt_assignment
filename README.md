# ROS Navigation System

This repository contains ROS (Robot Operating System) nodes and services for navigation and information retrieval.

## Contents

1. [Overview](#overview)
2. [Dependencies](#dependencies)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Pseudocode](#Pseudocode)


## Overview

The repository includes three main components:

1. **node_a**: Allows users to set and cancel navigation goals for a robot.
2. **node_b**: Provides a service to retrieve information about the last desired x and y positions.
3. **node_c**: Calculates and provides information about the distance and average velocity.

## Dependencies

- ROS (Melodic or later)
- Python 3
- Required ROS packages (if any, specify them here)

## Installation

1. Clone this repository:

    ```bash
    git clone https://github.com/MohsenKashefi/rt_second_assignment.git
    ```

# node a Pseudocode
initialize ROS node

initialize publishers, subscribers, and action client

# Main loop
while not ROS shutdown:
    # Prompt user for input
    prompt user for input

    # Check user input
    if input is 'd':
        # Get new goal coordinates from user
        get new goal coordinates from user

        # Create and send new goal to action server
        create and send new goal to action server

    elif input is 'q':
        # Cancel current goal if exists
        cancel current goal if exists

