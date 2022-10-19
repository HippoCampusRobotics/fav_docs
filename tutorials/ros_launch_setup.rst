ROS Launch Setup
################

.. attention::

   The following tutorial **is not meant as a step-by-step solution for the first assignment.** These are just **toy examples** to demonstrate how to use ROS and interact with the simulated BlueROV *in an easy to follow manner*. Therefore, we do not claim that these code snippets are complete and we use some funny names at times. Please do **not** copy-paste them. 

Before We Start
===============

So, before we start to create a super cool launch setup and have some super fancy nodes doing exciting stuff, lets take a step back and have another look on the keyboard-control setup from the setup instructions. Let us relaunch this setup and open just another terminal to run

.. code-block:: sh

   rqt_graph

Make sure to select Nodes/Topics(all) in the upper left corner and refresh the view. This should yield a graph like

.. image:: /res/images/keyboard_control_node_graph.png

You can see the different nodes :file:`/bluerov/keyboard`, :file:`/bluerov/mixer` and :file:`/bluerov/esc_commander` (we are not interested in the :file:`gazebo` node and will simply ignore it) inside ellipses and topics inside rectangles. Since all these nodes live inside the :file:`/bluerov` namespace and use relative topic names, everything has the :file:`/bluerov` prefix (more on this later).

The :file:`esc_commander` node is the interface between the ESCs which drive the thrusters and our ROS domain. It receives messages of the type :file:`fav_msgs/ThrusterSetpoint` on the :file:`thruster_setpoint` topic. That should be familiar to all of us from the previous tutorial and our dummy example with the :code:`setpoint_publisher.py`. The message definition can be looked up in :file:`~/fav/catkin_ws/src/fav/fav_msgs/msg/ThrusterSetpoint.msg` and is:

.. code-block::

   std_msgs/Header header
   float64[8] data

It contains the field :code:`data` that is an :code:`double` array of length 8. Each entry corresponds to a thruster. 

Now let's imagine the :code:`mixer` node in the above graph would not exist and the :file:`keyboard` node would have to publish messages of type :file:`fav_msgs/ThrusterSetpoint` directly. This would imply that the :file:`keyboard` node would have to know about the specific thruster configuration of our BlueROV to work. To move the vehicle forward when pressing :kbd:`W`, the :file:`keyboard` node would need to know that the first four motors are the only ones in horizontal direction and that they are configured in a way that all of them need to spin in positive direction to move the vehicle forward. 

To add a layer of abstraction we have the :file:`mixer` node. What :file:`keyboard` actually wants to do is to say "the user pressed :kbd:`W`, so move forward (i.e. set a positive value for thrust)" and from there on it is in the :file:`mixer`'s responsibility to translate this to actual setpoints for the specific thrusters that participate in the forward movement of the vehicle.

Basically, we divided a bigger problem into two smaller problems. In this case this can be especially handy, because also a controller we might program at some later stage, does not need to have knowledge of specific thrusters/actuators. It can directly output commands corresponding to the actuated degrees of freedom of the BlueROV. And since all degrees of freedom of the vehicle are actuated, we can control all degrees of directly |partying_face|.

Mathematically the :file:`mixer` node computes the following equation:

.. math:: 
   
   \begin{bmatrix}t_0\\\vdots\\t_7\end{bmatrix} = \boldsymbol{M} \begin{bmatrix}\textrm{roll}\\\textrm{pitch}\\\textrm{yaw}\\\textrm{thrust}\\\textrm{vertical thrust}\\\textrm{lateral thrust}\\0\\0\\\end{bmatrix},

where :math:`t_0` to :math:`t_7` are the direct thruster setpoints.

Having Fun with Open-Loop Control
=================================

Let us start where we have left off in the previous :ref:`tutorials/ros_package:ROS Package`  section.

We have a package called :code:`awesome_package`. And we have a node called :code:`setpoint_publisher.py`. Since we know about the :file:`mixer` now, we want to use it and have to modify our :file:`setpoint_publisher.py` to publish to the actuation topics instead of publishing directly to the :file:`thruster_setpoint` topic.

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy  # this is the python interface for ROS
   import math  # needed to use the trigonometric functions sin and cos
   from std_msgs.msg import Float64


   class MyFirstNode():
      def __init__(self):
         rospy.init_node("setpoint_publisher")
         self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                      Float64,
                                                      queue_size=1)

      def run(self):
         rate = rospy.Rate(30.0)

         while not rospy.is_shutdown():
               msg = Float64()
               t = rospy.get_time()
               msg.data = 0.5 * math.sin(t)
               self.vertical_thrust_pub.publish(msg)
               rate.sleep()


   def main():
      node = MyFirstNode()
      node.run()


   if __name__ == "__main__":
      main()

We do not created a new package or a new node, so we do not have to rebuild the workspace to apply the changes. But make sure you have saved the file after making these changes!

Make sure no nodes/launch setups are currently running. Otherwise stop them with :kbd:`Ctrl` + :kbd:`C` in the corresponding terminals. 

Start the simulation environment

.. code-block:: sh

   roslaunch fav_sim simulation.launch

Lastly start our :file:`setpoint_publisher` node:

.. code-block:: sh

   rosrun awesome_package setpoint_publisher.py

And you see... nothing. This will probably not be the last time things do not work out as expected. So let us investigate what might be the problem. Remember :code:`rqt_graph`? Great tool to see how nodes are connected (or not).

The command should yield something like this:

.. image:: /res/images/rqt_graph_setpoint_publisher_fail.png

Make sure to uncheck **Dead sinks** and **Leaf Topics**. Since the :file:`gazebo` and :file:`gazebo_gui` node are not relevant for our example, we can hide them by inserting :code:`-/gazebo,-/gazebo_gui` in the first text box. Also make sure **Nodes/Topics (all)** in the upper left corner and refresh the view.

Do you recognize how every node but our poor :file:`setpoint_publisher` lives inside the :file:`/bluerov` box? Now we will interact with namespaces for the first time. There are three distinct ways to declare topic names. They are either *global*, *relative*, or *private*. 

In our node we declared the topic name to be *relative*. But how can we tell? Because there is no leading :file:`/` or :file:`~`. 

.. code-block:: python
   :lineno-start: 9
   :linenos:

   self.vertical_thrust_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)

But what does it mean? It means the effective topic name will not necessarily be exactly :file:`vertical_thrust`. This depends on the namespace of our node. Since we did not specify any namespace during :code:`rosrun awesome_package setpoint_publisher.py`, the topic will be resolved as :file:`/vertical_thrust`. The :file:`mixer` node living inside the :file:`/bluerov` namespace subscribes to the relative topic :file:`vertical_thrust`. Due to the namespace this will resolve as :file:`/bluerov/vertical_thrust`. That is the reason why our node is not connected to the :code:`mixer`.

How to fix it, you may ask? We simply push our node into the :file:`/bluerov` namespace. This makes sense because our node is part of our BlueROV setup. Another 'fix' would be to prepend :file:`bluerov/` to the topic name of our publisher. But in this specific scenario I would rather call it botch. So let us push this node to the right namespace already! Just append :code:`__ns:=bluerov` to the :code:`rosrun` command.

.. code-block:: sh

   rosrun awesome_package setpoint_publisher __ns:=bluerov

This tells our node to live inside the :file:`bluerov` namespace. 

Refresh our view of :code:`rqt_graph` by clicking the refresh button in the upper left corner and you will see, we have a beautifully connected graph!

.. image:: /res/images/rqt_graph_setpoint_publisher_success.png


We can now admire our moving robot in the simulation:

.. image:: /res/images/gazebo_awesome_package.gif

By now we might get worried by the increasing number of needed terminal windows. Imagine we want to start additional nodes. Do we really need a separate terminal for each of them? Of course not! Launch files to the rescue!

 
Create A Launch Setup
*********************

.. todo:: The following sections need rework. They are not up to date for FaV 2022!

Create a new launchfile. You could name it :file:`first_simulation.launch` for example:

.. image:: /res/images/create_launchfile.gif

The launchfile has to start the :file:`motor_command_sender` node, the Gazebo simulation and spawns the BlueROV.

It could look like this:

.. code-block:: xml
   :linenos:

   <launch>
      <arg name="vehicle_name" default="bluerov" />

      <!-- start the gazebo simulator and an empty world -->
      <include file="$(find bluerov_sim)/launch/gazebo_base.launch" />
      
      <group ns="$(arg vehicle_name)">
        <!-- Spawn the vehicle. You can use the args to set the spawn pose-->
        <include file="$(find bluerov_sim)/launch/spawn_vehicle.launch">
           <!-- Set the position-->
           <arg name="x" value="0.0" />
           <arg name="y" value="0.0" />
           <arg name="z" value="-0.2"/>
           <!-- Set roll, pitch, yaw-->
           <arg name="R" value="0.0" />
           <arg name="P" value="0.0" />
           <arg name="Y" value="0.0" />
        </include>
        <!-- launch the motor_command_sender node-->
        <node name="motor_command_sender" pkg="awesome_package" type="motor_command_sender.py"/>
      </group>
   </launch>

To start the setup, run:

.. code-block:: sh

   roslaunch awesome_package first_simulation.launch

The result should look similar to:

Get Sensor Data
***************

At this point we know the basics of actuating the vehicle. But to know how we want to actuate the vehicle, we might depend on some sensor input. 

The BlueROV has a pressure sensor. The output of the pressure sensor is published under the :file:`pressure` topic.

Theoretically we could use the :file:`motor_command_sender.py` and modify its code to subscribe to the :file:`pressure` topic. But to keep things modular and separated, we add a new node to the :file:`awesome_package`. Let's name it :file:`depth_calculator.py`. 

.. note:: Keep in mind, you have to make every node executable! See :ref:`tutorials/ros_package:Write A Node`.

The source code might look like this:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy
   from sensor_msgs.msg import FluidPressure
   from std_msgs.msg import Float32


   def pressure_callback(pressure_msg, publisher):
      pascal_per_meter = 1.0e4
      # what kind of pressure data do we get? relative/absolute? What about
      # atmospheric pressure?
      depth = -pressure_msg.fluid_pressure / pascal_per_meter
      depth_msg = Float32()
      depth_msg.data = depth
      publisher.publish(depth_msg)


   def main():
      rospy.init_node("depth_calculator")
      depth_pub = rospy.Publisher("depth", Float32, queue_size=1)
      pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                       pressure_callback, depth_pub)
      rospy.spin()


   if __name__ == "__main__":
      main()

We can add this node to our launchfile by adding the following snippet:

.. code-block:: xml
   
   <node name="depth_calculator" pkg="awesome_package" type="depth_calculator.py" />

And launch the setup:

.. code-block:: sh

   roslaunch awesome_package first_simulation.launch

We can check that the nodes are properly connected in the :code:`rqt` node graph:

.. image:: /res/images/rqt_graph.png

And to inspect the data, we can plot it in :code:`rqt_multiplot` 

.. image:: /res/images/depth_multiplot.png

or use the :code:`rqt` topic monitor or simply in the command line:

.. code-block:: sh

   rostopic echo bluerov/depth

Names and Namespaces
====================


Namespaces
**********

The concept of names and namespaces is explained in detail in the `ROS Wiki <http://wiki.ros.org/Names>`__. 

You can start nodes or load parameters in namespaces (you can also have nested namespaces). This means that the namespace gets prepended to the node's name. We already used this in the above launchfile. Every node (and also every node in included launchfiles) inside the :code:`<group ns="$(arg vehicle_name)">` is launched inside a namespace. In this case the namespace's name is determined by the argument :code:`vehicle_name`. The default value of :code:`vehicle_name` is :code:`bluerov`. This means the name of the :code:`motor_command_sender` node launched in line 20 will become :code:`/bluerov/motor_command_sender`.

Names
*****

If you have a node subscribing or publishing to/from a topic, you have to specify the topic name. You can do this in three different ways:

Global
   .. code-block:: python

      pub = rospy.Publisher("/my_robot/pose", PoseStamped)
   
   A topic name with a leading :file:`/` will be resolved globally. This means that it does not matter if the node was launched in a namespace or not. The resulting topic name will be exactly :file:`/my_robot/pose`.

Relative
   .. code-block:: python

      pub = rospy.Publisher("position", PoseStamped)

   A topic without leading :file:`/` will be relative. This means that, if the node was launched in a namespace, the namespace will get prepended. So for example if the node was launched in the namespace :file:`my_robot`, the resolved topic name will become :file:`/my_robot/position`. In case the node was not launched inside any namespace, nothing will get prependended to the topic name. It will be just :file:`/position`.

Private
   .. code-block:: python

      rospy.init_node("my_controller")
      pub = rospy.Publisher("~debug", DebugMessage)

   Private topics are similar to relative ones. The topic name start with :file:`~`. The namespace will get prepended if it has been specified. Additionally the name of the node will also be prepended in any case. So if the node with the name :file:`my_controller` has been started in the namespace :file:`my_robot`, the resolved topic name will be :file:`/my_robot/my_controller/debug`. Without a namespace it would be :file:`/my_controller/debug`.

BlueROV
*******

You will only work with a single robot. Still it is nice to have things clean and start everything at least in the :file:`bluerov` namespace (as shown in the above example launchfile by launching your nodes inside the :code:`<group>`-tag with the ns attribute specified). 

Generally, avoid global topic names to avoid topic name collision if you do not have a very specific reason to use them. Example: if you have a controller subscribing to a setpoint topic, it might be a good idea to use a private name :code:`"~setpoint"`. This way you avoid topic name conflicts in case you have another controller also subscribing to a setpoint topic.
