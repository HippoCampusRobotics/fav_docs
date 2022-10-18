Concepts
========

.. .. image:: /res/images/gazebovsexperiment.png
..    :width: 90 %
..    :align: center
.. .. image:: /res/images/gazebo_depth.gif
..    :width: 40 %
..    :align: left
.. .. image:: /res/images/bluerov_depth.gif
..    :width: 36 %
..    :align: right

In this class, you will get in touch with the whole robotics pipeline. Starting from sensing, over planning to control and actuation, you will be developing suitable algorithms to tackle chosen challenges.

We will be using the `ROS <https://www.ros.org/>`_  framework. To evaluate the algorithms and solutions before deploying the real robot, we rely on :code:`gazebo` as simulation environment, where you can interact with the vehicle just as you would with a real vehicle.
This enables using the exact same code for both simulation and real world experiment.

.. image:: /res/images/gazebo_vs_experiment.gif

Keywords
########

Gazebo
   `Gazebo <http://gazebosim.org/>`_ is the standalone simulator that computes the physical behaviour of the objects in the simulation. It also generates sensor data or renders camera images of simulated camera sensors. :code:`gazebo` itself is independent of ROS. Maybe it is the easiest to view :code:`gazebo` as a drop-in replacement for the real world robot, so we can work on our algorithms without beeing in the lab all day.

ROS
   ROS is the framework we use in this course. So probably most of the time (if not all) is spent on developing/implementing algorithms and letting them interact via the communication capabilities ROS provides. Imagine ROS as some advanced message passing library. So instead of writing a single rather complex program, that handles *all* of our problems, we can use ROS to write multiple small programs that can communicate with each other via messages and solve only particular tasks.


Hardware Architecture
#####################

The following image depicts our setup for the real robot.

.. image:: /res/images/architecture_with_bluerov.svg

You will develop your algorithms in ROS and make use of the publisher/subscriber system, which allows us to run different functionalities on different machines. In fact ROS abstracts the communication in a way that we as users do not even recognize whether or not our programs/nodes communicate across different machines (computers) in the same network.

When working in the lab, your own laptop will be connected to our network, and you will be able to access all messages running in the ROS network.
The robot is connected to the network via its tether as well.

The BlueROV2 is equipped with an onboard computer (Raspberry Pi 4B) and a localization module (another Raspberry Pi 4B + Camera). Technically, we could run all code on here if we wanted to operate the underwater vehicle fully autonomously. However, the tether allows us to use external computational power and being able to plot data in realtime makes debugging so much easier.

The eight motors each have a motor controller (Electronic Speed Controller or ESC) that is connected to the onboard computer via a microcontroller.

All components inside the blue area belong physically to the BlueROV robot. For simulation the components in this area are replaced by the :code:`gazebo` simulator.


Localization Module
-------------------

Additionally, the lower enclosure of the robot contains our localization module consisting of a camera and a second RaspberryPi, which is connected to the first one via Ethernet.
The camera is directed at the tank's floor, which is equipped with AprilTags at known locations. The detected AprilTags in the camera image can be used to localize the robot relative to the tags. More on this `here <https://ras.papercept.net/proceedings/IROS20/1220.pdf>`_.

 
