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

We will be using the ROS framework in combination with PX4-Autopilot firmware running on the vehicle. In simulation, this is done via Software-in-the-Loop (SITL), where you can interact with the vehicle just as you would with a real vehicle.
This enables using the exact same code for both simulation and real world experiment.

.. image:: /res/images/gazebo_vs_experiment.gif

Keywords
########

Gazebo
   `Gazebo <http://gazebosim.org/>`_ is the simulator that computes the physical behaviour of the objects in the simulation. It also generates sensor data or renders camera images of simulated camera sensors.

PX4
   `PX4 Autopilot <https://px4.io/>`_ is the firmware that runs on the (real or simulated) vehicle. This open-source firmware is mainly used in the field of aerial vehicles, but can also be used to operate underwater vehicles. In general, it provides low-level, realtime capable and high frequent controllers, for example attitude or rate controllers.
   For Formulas and Vehicles, we use PX4 in a passthrough mode. This means, we directly control each  motor from within ROS.

   In addition to its controllers, PX4 also offers extensive estimation libraries and safety features. We do use PX4 for state estimation, for example for estimating the vehicle's orientation from IMU measurements.

MAVLink
   `MAVLink <https://mavlink.io/en/>`_ is a communication protocol for unmanned vehicles. Communication with the PX4-Autopilot firmware has to be done via MAVLink messages.

MAVROS
   MAVROS is a bridge between ROS messages and MAVLink messages. We develop our programs and algorithms in ROS. MAVROS provides us with an interface so we do not need to get in touch with MAVLink messages directly. Messages we send to MAVROS get translated and forwarded as MAVLink messages to the PX4-Autopilot. Vice versa for the messages sent by the PX4-Autopilot.

ROS
   ROS is the framework we use in this course. So probably most of the time (if not all) is spent on developing/implementing algorithms and letting them interact via the communication capabilities ROS provides.


Hardware Architecture
#####################

The following image depicts our setup for the real robot.

.. image:: /res/images/architecture_with_bluerov.svg

You will develop your algorithms in ROS and make use of the publisher/subscriber system, which allows us to run different functionalities on different machines.

When working in the lab, your own laptop will be connected to our network, and you will be able to access all messages running in the ROS network.
The robot is connected to the network via its tether as well.

The BlueROV2 is equipped with a Flight Computer (or Flight Control Unit (FCU), or Autopilot).
We use the `Pixhawk 4 <https://docs.px4.io/master/en/flight_controller/pixhawk4.html>`_.
Onboard this flight computer runs the PX4-Autopilot firmware.

The flight computer can be interfaced with a so-called *companion computer*, which can be any single board computer. We use a RaspberryPi 4B on which we run ROS.
Technically, we could run all code on here if we wanted to operate the underwater vehicle fully autonomously.
However, the tether allows us to use external computational power and being able to plot data live makes debugging so much easier.

The eight motors each have a motor controller (Electronic Speed Controller or ESC) that is connected to the flight computer. Typically, low-level control is done on the flight computer and actuator commands are computed here.
For the sake of simplicity, during this class, we will be computing motor commands within ROS. The commands will then be directly passed through the flight computer to the actuators.


Localization Module
-------------------

Additionally, the lower enclosure of the robot contains our localization module consisting of a camera and a second RaspberryPi, which is connected to the first one via Ethernet.
The camera is directed at the tank's floor, which is equipped with AprilTags at known locations. The detected AprilTags in the camera image can be used to localize the robot relative to the tags. More on this `here <https://ras.papercept.net/proceedings/IROS20/1220.pdf>`_.

 
