Final Project
#############

.. attention::

   You will need to update our repositories: :ref:`updating`.

Install Dependencies
====================

The template provided for this assignment has additional dependencies.
We install them with

.. code-block:: console

   $ TODO - RVIZ progress bar installieren


Template Package
================

Get the Template
****************

.. code-block:: console

   $ cd ~/fav/ros2/src && \
   git clone https://github.com/FormulasAndVehicles/final_project.git


Remember to build and source your workspace.

Launch the Project
==================

We will use RVIZ for visualization during the final project. 
Start the simulation with Gazebo in headless mode: 

.. code-block:: console

   $ ros2 launch fav simulation.launch.py vehicle_name:=bluerov00 start_gui:=false

Launch the final project:

.. code-block:: console

   $ ros2 launch final_project final_project.launch.py vehicle_name:=bluerov00 scenario:=1 use_sim_time:=true

The scenario can be chosen using the launch argument :code:`scenario`.

RViz should open and look like this:

.. image:: /res/images/screenshot_before_start.png

Now, to start the algorithm, you need to call the start service:

.. code-block:: console

   $ ros2 service call /bluerov00/scenario_node/start std_srvs/srv/Trigger

You can now see the viewpoints and obstacles as well as the occupancy grid map.
Once a path is found, the BlueROV automatically drives to the next viewpoint (so far using our very simple solution).

It should look like this:

.. image:: /res/images/screenshot_progress.png

For a viewpoint to be registered as 'visited', the robot needs to be in close proximity for a while. The progress is displayed in the overlay chart on the left.

The needed time from start of computation to completion of all viewpoints will be displayed:

.. image:: /res/images/screenshot_finished.png


Restarting your code
********************

When you want to rerun your algorithm, simply restart the :code:`final_project.launch.py` launch file and call the start service.


Scenario Description
====================

We provide you with 3 different scenarios. Furthermore, feel free to construct your own scenarios.
You can find the scenario files here: :file:`config/scenario_X.yaml`

A scenario is described by obstacles and viewpoints.
Each obstacle is a polygon with n corner points, described by their x and y coordinate. 
We use a pose to describe each viewpoint (position + quaternion desribing the orientation). However, only the x and y position, as well as the yaw angle are relevant.

Provided Nodes
==============

Mapper
******
This node computes an occupancy grid map.
In the file :file:`config/mapping_params.yaml`, you can change the discretization.

All obstacles included in the scenario description will automatically be included in the grid map.

Additionally, we have already implemented a safety margin around all obstacles.
Since the BlueROV's real size is not necessary identical with the grid cells' size, the obstacles need to be inflated and additional grid cells marked as occupied in order to avoid collision.

In order to adjust this inflation size, have a look at this node's source code.

Apart from this, you should not need to touch this node.


Path Planner
************

Path Follower
*************

Position Controller
*******************

Yaw Controller
**************


Rviz
====

If you are using a 4k monitor, the RViz overlays showing the status and progress of the scenario might not be displayed correctly.

You can change the size of the circular progress display in :file:`config/rviz.rviz` here:

.. code-block:: 
   :emphasize-lines: 26

   - Class: rviz_2d_overlay_plugins/PieChartOverlay
   Enabled: true
   Name: Progress
   Topic:
      Depth: 5
      Durability Policy: Volatile
      History Policy: Keep Last
      Reliability Policy: Reliable
      Value: /bluerov00/scenario_node/viewpoint_progress
   Value: true
   auto color change: true
   background color: 0; 0; 0
   backround alpha: 0
   clockwise rotate direction: true
   foreground alpha: 0.699999988079071
   foreground alpha 2: 0.4000000059604645
   foreground color: 255; 255; 255
   left: 10
   max color: 0; 255; 0
   max color change threthold: 0.75
   max value: 1
   med color: 255; 255; 0
   med color change threthold: 0.10000000149011612
   min value: 0
   show caption: true
   size: 80
   text size: 14
   top: 80

200 could be a good value for 4k resolution.