.. rt_assignment2 documentation master file, created by
   sphinx-quickstart on Sat Apr 20 13:41:23 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to rt_assignment2's documentation!
==========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


node_a
**************************************
his code implements node_a which handles goal setting and cancellation for a robot using ROS (Robot Operating System).

Subscribers
-----------
- /odom: Subscribes to the odometry information.

Publishers
----------
- /pos_vel: Publishes the current position and velocity information.

Actions
-------
- /reaching_goal: Sends goals to the action server to reach a specified position.


----------------

publish_position_and_velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Publishes the current position and velocity information.

:param msg: Odometry message containing position and velocity information.
:type msg: nav_msgs.msg.Odometry
:param pos_vel_publisher: Publisher for position and velocity information.
:type pos_vel_publisher: rospy.Publisher

get_user_input
~~~~~~~~~~~~~~
Retrieves user input with a prompt.

:param prompt: Prompt to display to the user.
:type prompt: str
:return: User input
:rtype: str

get_new_goal
~~~~~~~~~~~~

Retrieves a new goal from user input.

:return: New goal and its coordinates
:rtype: tuple(assignment_2_2023.msg.PlanningGoal, tuple(float, float))

handle_goal_commands
~~~~~~~~~~~~~~~~~~~~

Handles goal commands from the user.

main
~~~~

Main function to run the goal handler.

===========================

.. automodule:: scripts.node_a
  :members:

.. _scripts.node_b:

node_b
===========================

This code implements Node B.

Subscriber
----------
- /node_b/pose: Subscribes to the pose topic.

Service
-------
- /input: Provides the last desired x and y positions.


Function Details
----------------

handle_input_request
~~~~~~~~~~~~~~~~~~~~

Callback function for the 'input' service.

Retrieves the last desired x and y positions from ROS parameters and constructs a response message containing these positions.

:param request: Request message sent to the service.
:return: Response message containing the last desired x and y positions.
:rtype: assignment_2_2023.srv.InputResponse

provide_input_service
~~~~~~~~~~~~~~~~~~~~~

Initializes the 'input' service and keeps the node running to provide the service.


.. module:: scripts.node_b

.. automodule:: scripts.node_b
   :members:
   :undoc-members:
   :show-inheritance:
   
.. _scripts.node_c:

node_c
===========================

Subscribers
-----------
- /pos_vel: Subscribes to the position and velocity information.

Services
--------
- info_service: Provides information about distance and average velocity.

Function Details
----------------

get_distance_and_average_velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Callback function to compute distance and average velocity.

:param msg: Velocity message containing position and velocity information.
:type msg: assignment_2_2023.msg.Vel

get_values
~~~~~~~~~~
Service handler function to provide distance and average velocity.

:param _: Empty request, as no input is required.
:type _: No input type.
:return: Response containing distance and average velocity.
:rtype: assignment_2_2023.srv.Ave_pos_velResponse

main
~~~~
Main function to initialize the ROS node and set up subscribers and services.

.. module:: scripts.node_c

.. automodule:: scripts.node_c
   :members:
   :undoc-members:
   :show-inheritance:
