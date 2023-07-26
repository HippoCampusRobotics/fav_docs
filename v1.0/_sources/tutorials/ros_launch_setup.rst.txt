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

Basically, we divided a bigger problem into two smaller problems. In this case, this can be especially handy because also a controller we might program at some later stage does not need to have knowledge of specific thrusters/actuators. It can directly output commands corresponding to the actuated degrees of freedom of the BlueROV. And since all degrees of freedom of the vehicle are actuated, we can control all degrees of directly |partying_face|.

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

   roslaunch fav_sim gazebo_apriltag_tank_world.launch

spawn the vehicle

.. code-block:: sh

   roslaunch fav_sim spawn_vehicle.launch

and lastly start our :file:`setpoint_publisher` node:

.. code-block:: sh

   rosrun awesome_package setpoint_publisher.py

And you see... nothing. This will probably not be the last time things do not work out as expected. So let us investigate what might be the problem. Remember :code:`rqt_graph`? Great tool to see how nodes are connected (or not).

The command should yield something like this:

.. image:: /res/images/rqt_graph_setpoint_publisher_fail.png

Make sure to uncheck **Dead sinks** and **Leaf Topics**. Since the :file:`gazebo` and :file:`gazebo_gui` node are not relevant for our example, we can hide them by inserting :code:`-/gazebo,-/gazebo_gui` in the first text box. Also make sure **Nodes/Topics (all)** is selected in the upper left corner and refresh the view.

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
=====================

Create a new launchfile. You could name it :file:`setpoint.launch` for example:

.. image:: /res/images/create_launchfile.gif

It could look like this:

.. code-block:: xml
   :linenos:

   <launch>
      <arg name="vehicle_name" default="bluerov" />

      <!-- start the simulation -->
      <include file="$(find fav_sim)/launch/simulation.launch" pass_all_args="true" />

      <group ns="$(arg vehicle_name)">
         <!-- start the setpoint publisher node -->
         <node name="setpoint_publisher" pkg="awesome_package" type="setpoint_publisher.py" />
      </group>
      
      <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" />
   </launch>

Explanation
===========

Let's take a detailed look what we have here.

Arguments
*********

.. code-block:: xml
   :lineno-start: 2
   :linenos:

   <arg name="vehicle_name" default="bluerov" />

Declares an argument named :code:`vehicle_name` and assigns the default value :code:`"bluerov"`. We will use this argument to set the namespace of the nodes to be launched. To overwrite this argument without having to modify the launch file, we can simply append :code:`vehicle_name:="A_NEW_VALUE"` to the :code:`roslaunch` command.

Include Files
*************

.. code-block:: xml
   :lineno-start: 5
   :linenos:

   <include file="$(find fav_sim)/launch/simulation.launch" pass_all_args="true" />

We can include other launch files. It is literally the same as copy pasting the content of the specified file right inside our own launch file. Furthermore, we have the special syntax :code:`$(find fav_sim)` here. We do not have to know the full path to the launch file. We can use :code:`$(find)` to get the path to ros packages. In case the :code:`pass_all_args` attribute is set to :code:`true`, all arguments in our launch file get passed to the included launch file. Otherwise this would not be the case.

Groups and Nodes
****************

.. code-block:: xml
   :lineno-start: 7
   :linenos:

   <group ns="$(arg vehicle_name)">
      <!-- launch the motor_command_sender node-->
      <node name="setpoint_publisher" pkg="awesome_package" type="setpoint_publisher.py" />
   </group>

Two things here. We can declare groups and assign a namespace to everything that is inside this group by settings the :code:`ns` attribute. To use the arguments we have declared in the launch file or pass in via the command line, we use :code:`$(arg parameter_name)` so in our case :code:`$(arg vehicle_name)`. To start the :code:`setpoint_publisher` node, we use the :code:`<node>` tag. The :code:`name` attribute overwrites the node's name set in the sourcode by :code:`rospy.init_node("setpoint_publisher")`. :code:`pkg` is the name of the package where the node is located. And :code:`type` is the file name of the executable.

.. code-block:: xml
   :lineno-start: 12
   :linenos:

   <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" />

This starts the :code:`rqt_graph` tool directly in our launch setup. This way we do not have to start it in another terminal to see the nodegraph. 

Launch the Setup
================

So this launch file produces the exact same setup we have created in the section :ref:`tutorials/ros_launch_setup:having fun with open-loop control` before. The advantage is, we can start it with a single command:

.. code-block:: sh

   roslaunch awesome_package setpoint.launch

Really looks the same, doesn't it? Now stop everything and try to assign the :code:`vehicle_name` parameter from the command line.

.. code-block:: sh

   roslaunch awesome_package setpoint.launch vehicle_name:=klopsi

Everything will still be connected just fine. The only difference is, that every node is running inside the :file:`/klopsi` namespace.

Taking the Next Step
====================

We can also pass arguments to the launch file that are not declared in the file we are launching directly. Remember that we set :code:`pass_all_args` to true when including :file:`simulation.launch`? Inside :file:`simulation.launch` the file :file:`spawn_vehicle.launch` is included and all arguments are passed as well. 

.. image:: /res/images/spawn_vehicle.png

There are arguments :code:`x`, :code:`y` and :code:`z` declared for the spawning position of the vehicle and :code:`R`, :code:`P` and :code:`Y` for the orientation. We can pass arguments all the way down to this launch file. So we can modify the spawning position of the vehicle by running

.. code-block:: sh

   roslaunch awesome_package setpoint.launch x:=4 z:=-3

Maybe it is necessary to rotate the camera inside gazebo to find the BlueROV in its new position.

Get Sensor Data
===============

At this point we know the basics of actuating the vehicle. But to know how we want to actuate the vehicle, we might depend on some sensor input. 

The BlueROV has a pressure sensor. The output of the pressure sensor is published under the :file:`pressure` topic inside the vehicle's namespace. So by default the topic name will be :file:`/bluerov/pressure`.

Theoretically, we could use the :file:`setpoint_publisher.py` and modify its code to subscribe to the :file:`pressure` topic. But to keep things modular and separated, we add a new node to the :file:`awesome_package`. Let's name it :file:`depth_calculator.py`. You could argue that having a complete program only calculating the depth coordinate of the vehicle from pressure data might seem like a bit overkill. But let's see the :file:`depth_calculator` as some specific case of a state estimation. And this can get complex very quickly. Therefore, it is a good idea to solve separate problems in separate nodes.

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

.. hint:: Confused on how you should know what the structure of a FluidPressure message is and how to access its data? Simply search for "ros fluidpressure" and you will find the `message definition <http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html>`_. Message fields are accessed by a dot operator.

We can add this node to our launchfile by adding the following snippet inside the :code:`<group>`` tag:

.. code-block:: xml
   
   <node name="depth_calculator" pkg="awesome_package" type="depth_calculator.py" />

And launch the setup:

.. code-block:: sh

   roslaunch awesome_package setpoint.launch

We can check that the nodes are properly connected in the graph:

.. image:: /res/images/rqt_graph.png

.. note:: Refresh the node graph with the refresh button in the upper left corner to make sure the graph is up-to-date.

And to inspect the data, we can plot it in :code:`rqt_multiplot` 

.. image:: /res/images/depth_multiplot.png

or use the :code:`rqt` topic monitor or simply in the command line:

.. code-block:: sh

   rostopic echo bluerov/depth

We can see that the data is noisy. And in the real world data is *always* noisy. But depending on the scenario, there is a wide range of filtering methods available. One could compute a moving average over the last :math:`n` data points, a very simple software first order lowpass filter or maybe even something more advanced like a Kalman filter or a particle filter. But the possibilites are of course not limited to those approaches.

The Missing Link
================

So now we have a :code:`depth_calculator` computing the depth of the BluerROV in some way and we have a :code:`setpoint_publisher` publishing vertical thrust values to move the BlueROV. What about renaming the :code:`depth_calculator` to :code:`depth_estimator` and make the :code:`setpoint_publisher` a :code:`depth_controller`? Maybe a :code:`depth_controller` should subscribe to a setpoint topic as well as to the current depth?
