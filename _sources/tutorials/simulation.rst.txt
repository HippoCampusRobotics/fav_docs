Simulation
##########


(Minimal) Example
=================

Let us start where we have left off in the previous :ref:`tutorials/ros:ROS`  section:

We have a package called :code:`awesome_package`. And we have node called :code:`motor_command_sender.py`. To get a working example, we have to add two things. The first thing is, we need to modify the node slightly, to *arm* the vehicle. This means, we have to tell the PX4-Autopilot that it is now allowed to actuate the motors. The other thing we have to take care of, is the Gazebo part. So we need to start Gazebo and spawn our model of the BlueROV.


Create A Launch Setup
*********************

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

To start a the setup run:

.. code-block:: sh

   roslaunch awesome_package first_simulation.launch

The result should look similar to:

.. image:: /res/images/gazebo_awesome_package.gif

Get Sensor Data
***************

At this point we know the basics of actuating the vehicle. But to know how we want to actuate the vehicle, we might depend on some sensor input. 

The BlueROV has a pressure sensor. The output of the pressure sensor is published under the :file:`pressure` topic.

Theoretically we could use the :file:`motor_command_sender.py` and modify it's code to subscribe to the :file:`pressure` topic. But to keep things modular and separated, we add a new node to the :file:`awesome_package`. Let's name it :file:`depth_calculator.py`. 

.. note:: Keep in mind, you have to make every node executable! See :ref:`tutorials/ros:Write A Node`.

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

You can start nodes or load parameters in namespaces (you can also have nested namespaces). This means the namespace gets prepended to the node's name. We already used this in the above launchfile. Every node (and also every node in included launchfiles) inside the :code:`<group ns="$(arg vehicle_name)">` is launched inside a namespace. In this case the namespace's name is determined by the argument :code:`vehicle_name`. The default value of :code:`vehicle_name` is :code:`bluerov`. This means the name of the :code:`motor_command_sender` node launched in line 20 will become :code:`/bluerov/motor_command_sender`.

Names
*****

If you have a node subscribing or publishing or subscribing to/from a topic, you have to specify the topic name. You can do this in three different ways:

Global
   .. code-block:: python

      pub = rospy.Publisher("/my_robot/pose", PoseStamped)
   
   A topic name with a leading :file:`/` will be resolved globally. This means it does not matter if the node was launched in a namespace or not. The resulting topic name will be exactly :file:`/my_robot/pose`.

Relative
   .. code-block:: python

      pub = rospy.Publisher("position", PoseStamped)

   A topic without leading :file:`/` will be relative. This means if the node was launched in a namespace, the namespace will get prepended. So for example if the node was launched in the namespace :file:`my_robot`, the resolved topic name will become :file:`/my_robot/position`. In case the node was not launched inside any namespace, nothing will get prependended to the topic name. It will be just :file:`/position`.

Private
   .. code-block:: python

      rospy.init_node("my_controller")
      pub = rospy.Publisher("~debug", DebugMessage)

   Private topics are similar to relative ones. The topic name start with :file:`~`. The namespace will get prepended, if it has been specified. Additionally the name of the node will also be prepended in any case. So if the node with the name :file:`my_controller` has been started in the namespace :file:`my_robot`, the resolved topic name will be :file:`/my_robot/my_controller/debug`. Without a namespace it would be :file:`/my_controller/debug`.

BlueROV
*******

You will only work with a single robot. Still it is nice to have things clean and start everything at least in the :file:`bluerov` namespace (as shown in the above example launchfile by launching your nodes inside the :code:`<group>`-tag with the ns attribute specified). Generally avoid global topic names to avoid topic name collision, if you do not have a very specific reason to use them. Example: if you have a controller subscribing to a setpoint topic, it might be a good idea to use a private name :code:`"~setpoint"`. This way you avoid topic name conflicts in case you have another controller also subscribing to a setpoint topic.