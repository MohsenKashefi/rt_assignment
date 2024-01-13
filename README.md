# ROS Navigation System

This repository contains ROS (Robot Operating System) nodes and services for navigation and information retrieval.

## Contents

1. [Overview](#overview)
2. [Dependencies](#dependencies)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Code Structure](#code-structure)
6. [Additional Information](#additional-information)

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
    git clone https://github.com/<your_username>/<your_repository>.git 
    ```.

## Usage

# set_target_client.py

# Initialize ROS node
initialize ROS node

# Initialize publishers, subscribers, and action client
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

