ROS Resources
#################

.. note::
   This section tries to give you an overview of additional resources and tools you might need when working with ROS. This is by no means complete, and we encourage you to explore what's out there on your own.


You can find the ROS wiki `here <http://wiki.ros.org/>`_.




ROS Tutorials
=============
The official ROS `tutorials <http://wiki.ros.org/ROS/Tutorials>`__  are a very good place to start if you've never used ROS before and got some time on your hands.
There are C++ and Python versions for coding examples. Since we will be using Python in this class, you can skip C++ specific chapters.
However, if you're already confident in C++ (more so than in Python), feel free to use C++ instead. 

.. note::
   There are a lot of other (beginner's) tutorials for ROS online. If the official ROS tutorials aren't for you, just have a look on google.

Commandline Tools
==================

There are a lot of `commandline tools <http://wiki.ros.org/ROS/CommandLineTools>`_. 
Apart from roslaunch, tools that you might find helpful include rostopic, rosbag, rosparam and roscd. 
When encountering problems with your ROS setup, roswtf can be very useful.


Visualization Tools
===================

Once you start working with ROS and are writing your own nodes, you will want to be able to visualize data and analyze interaction between nodes.


rqt
***
`rqt <http://wiki.ros.org/rqt>`_  consists of various GUI tools, almost all of them are useful. We make use of the Topic Monitor and rqt `Multiplot <https://github.com/ANYbotics/rqt_multiplot_plugin>`_  the most. For testing, the Message Publisher is also very useful.

#. To install Multiplot, run

   .. code-block:: sh

      sudo apt install ros-noetic-rqt-multiplot

#. To tell rqt about this new plugin, run
   
   .. code-block:: sh

      rqt --force-discover

Now, you will be able to use the Multiplot plugin from the rqt GUI.

RViz
****
`RViz <http://wiki.ros.org/rviz>`_  is an awesome tool for 3D visualization, but can take some time getting used to it. RViz can visualize different robot coordinate frames, for coordinate transformations in ROS, look here: 
:ref:`resources:Coordinate transformations`

Recording and playing back data
===============================
`ROSbags <http://wiki.ros.org/rosbag>`_ are an awesome functionality of ROS, trust me. Use them. There also is a ROS `tutorial <http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data>`__ on how to record and play back data.

In case you want to then plot your data in, for example, Matlab, converting your ROSbags to \*.csv files will be useful. 
I like `this tool <https://github.com/AtsushiSakai/rosbag_to_csv>`_ for that purpose, but there are other options out there.

We now also have a tutorial available in the section :ref:`tutorials/recording_rosbags:Recording Data Using Bag Files`.
   
Coordinate Transformations
==========================
ROS uses the tf transformations tool to keep track of coordinate frames and their relationships over time. The second and newest generation of the transform library is `tf2 <http://wiki.ros.org/tf2>`_ .

If you want to transform vectors by hand using rotation matrices or quaternions, you might find the `transformations (Python) API <http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html>`_ helpful.

The :code:`tf.transformations` library also helps you deal with quaternions by providing various transformation options, for example to Euler angles.


Messages
========
There's a pre-defined ROS message for almost everything, trust me. 
Standard messages representing primitive data types and other basic message constructs, such as multiarrays, can be found in `std_msgs <http://wiki.ros.org/std_msgs>`_.
The package `common_msgs <http://wiki.ros.org/common_msgs>`_ includes the most common generic robot-specific message packages, such as `geometry_msgs <http://wiki.ros.org/geometry_msgs?distro=noetic>`_,  `sensor_msgs <http://wiki.ros.org/sensor_msgs?distro=noetic>`_ and `nav_msgs <http://wiki.ros.org/nav_msgs?distro=noetic>`_.
When you google a message name, you'll easily find the API (`example <http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html>`_ for a :code:`sensor_msgs/Imu.msg` ).

However, if you do find yourself needing to define your own message type, there is a ROS `tutorial <http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg>`__ on that, too.

ROS Logger
==========
Of course you can use simple :code:`print()` commands. However, ROS has a `logger <http://wiki.ros.org/rospy/Overview/Logging>`_ that you can (should) use for printing messages to your console if they are not for debugging purposes only. There is, of course, a `tutorial <http://wiki.ros.org/rospy_tutorials/Tutorials/Logging>`_.


Installing additional Packages
==============================
Released packages can be installed with aptitude and don't have to be build from source. Use:

.. code-block:: sh

      sudo apt install ros-noetic-<package>

Additional Stuff
================
Unit and coordinate conventions used within ROS:
`REP 103 <https://www.ros.org/reps/rep-0103.html>`_ 

