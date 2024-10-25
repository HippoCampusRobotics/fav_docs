ROS Resources
#################


This section gives you an overview of additional resources and tools you might need when working with ROS. This is by no means complete, and we encourage you to explore what's out there on your own.


ROS Tutorials
=============

If you're interested in even more in-depth information on ROS2, the official ROS `tutorials <https://docs.ros.org/en/jazzy/Tutorials.html>`__  are a very good place to start.

There are C++ and Python versions for coding examples. Since we will be using Python in this class, you can skip C++ specific chapters.


ROS Concepts
============

If you don't quite understand the underlying concepts of ROS yet: have a look `here <https://docs.ros.org/en/jazzy/Concepts.html>`_. 


Visualization Tools
===================

Once you start working with ROS and are writing your own nodes, you will want to be able to visualize data and analyze interaction between nodes.

Plotjuggler
***********
We have a tutorial on this now! Go to :ref:`tutorials/real_time_plotting:real-time plotting`.


RViz
****
`RViz <https://github.com/ros2/rviz>`_  is an awesome tool for 3D visualization, but can take some time getting used to it. RViz can visualize different robot coordinate frames, for coordinate transformations in ROS, look here: 
:ref:`00_main_toc/resources:Coordinate transformations`

Recording and playing back data
===============================
`ROSbags <http://wiki.ros.org/rosbag>`_ are an awesome functionality of ROS, trust me. Use them. 

We are working on a tutorial in the section :ref:`tutorials/bag_file_plotting:bag file plotting`.
   
Coordinate Transformations
==========================
ROS uses the tf transformations tool to keep track of coordinate frames and their relationships over time. The second and newest generation of the transform library is `tf2 <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html>`_ .

The :code:`tf.transformations` library also helps you deal with quaternions by providing various transformation options, for example to Euler angles.


Messages
========
There's a pre-defined ROS message for almost everything, trust me. 
Standard messages representing primitive data types and other basic message constructs, such as multiarrays, can be found in `std_msgs <https://docs.ros2.org/foxy/api/std_msgs/index-msg.html>`_.
The package `common_interfaces <https://github.com/ros2/common_interfaces>`_ includes the most common generic robot-specific message packages, such as `geometry_msgs <https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html>`_,  `sensor_msgs <https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html>`_ and `nav_msgs <https://docs.ros2.org/foxy/api/nav_msgs/index-msg.html>`_.
When you google a message name, you'll easily find the API (`example <http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html>`_ for a :code:`sensor_msgs/Imu.msg` ).

However, if you do find yourself needing to define your own message type, there is a ROS `tutorial <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html>`__ on that, too.

ROS Logger
==========
Of course you can use simple :code:`print()` commands. However, ROS has a `logger <https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html>`_ that you can (should) use for printing messages to your console if they are not for debugging purposes only.


Installing additional Packages
==============================
Released packages can be installed with aptitude and don't have to be build from source. Use:

.. code-block:: console

   $ sudo apt install ros-jazzy-<package>

Additional Stuff
================
Unit and coordinate conventions used within ROS:
`REP 103 <https://www.ros.org/reps/rep-0103.html>`_ 

