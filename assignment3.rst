Assignment 3
############

.. note:: 

   Remember to regularly update your local repositories as described in the previous section. In case we make major changes, we'll send a slack reminder to update.


Get the Code
============

You will need to update your :code:`bluerov_sim` package, for example by:

.. code-block:: sh

   roscd bluerov_sim && git pull

You will also need the :code:`apriltag_ros` package:

.. code-block:: sh

   sudo apt-get install ros-melodic-apriltag-ros

We have prepared a :code:`range_sensor` package that will publish the range measurements in a topic :code:`/ranges`:

.. code-block:: sh

   git clone https://github.com/HippoCampusRobotics/fav_range_sensor.git ~/fav/catkin_ws/src/range_sensor

Don't forget to rebuild your catkin workspace after downloading these packages.

We have made some adjustments to the PX4-Autopilot firmware running on the Flight Control Unit as well, so we need to update this:

.. code-block:: sh

   cd ~/fav/fav_PX4-Autopilot && git pull

And to rebuild the code, execute in the firmware's directory:

.. code-block:: sh

   DONT_RUN=1 make clean 
   DONT_RUN=1 make -j1 px4_sitl gazebo_uuv_bluerov2_heavy

.. note::

   If you got an internal compiler error last time you built the firmware, this will probably happen again. Just repeat the build command a few times until it works.


Range Sensor in Simulation
==========================

We are not actually using simulated camera images and running the AprilTag algorithm in simulation. Rather, we have written a Gazebo Plugin that simulates the range measurements to the given four anchors. This has the benefit of reducing the computational burden.
TODO



Some Final Remarks
==================

This assignment, you will do different things, including implementing a self-localization algorithm for the BlueROV2 and a controller to autonomously navigate inside the tank. Think about your code- and package-structure to ensure (re-)usability.

.. attention::

   Please do not change code in our repositories, namely :code:`bluerov_sim` and :code:`range_sensor`. There are some adjustments missing for the experiment that we will be working on, so you will have to be able to pull our uploaded code from Github.
   Instead, create your own packages.


