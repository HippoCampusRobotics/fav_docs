Simulation
##########

Concept
=======

When we are working with the simulation, we have interactions between several components.

Gazebo
   Gazebo is the simulator, that computes the physical behaviour of the objects in the simulation. It also generates sensor data or renders camera images of simulated camera sensors.

PX4
   PX4-Autopilot is the firmware that runs on the (real or simulated) vehicle. Mainly it is used in the field of aerial vehicles, but can also be used to operate underwater vehicles. In general it provides low-level, realtime capable and high frequent controllers, for example attitude or rate controllers. For Formulas and Vehicles we use PX4 in a passthrough mode. This means, we directly control each  motor from within ROS via the :file:`mavros/setpoint_motor/setpoint` topic. We do not rely on the firmware's onboard controllers.

MAVLink
   MAVLink is a communication protocol for unmanned vehicles. Communication with the PX4-Autopilot firmware has to be done via MAVLink messages.

MAVROS
   A bridge between ROS messages and MAVLink messages. We develop the programs in ROS. MAVROS provides us with an interface so we do not need to get in touch with MAVLink messages directly. Messages we send to MAVROS get translated and forwarded as MAVLink messages to the PX4-Autopilot. Vice versa for the messages sent by the PX4-Autopilot.

ROS
   ROS is the framework we use in this course. So probably most of the time (if not all) is spent on developing/implementing algorithms and letting them interact via the communication capabilities ROS provides.

(Minimal) Example
=================

Let us start where we have left off in the previous :ref:`next_steps/ros:ROS`  section:

We have a package called :code:`awesome_package`. And we have node called :code:`motor_command_sender.py`. To get a working example, we have to add two things. The first thing is, we need to modify the node slightly, to *arm* the vehicle. This means, we have to tell the PX4-Autopilot that it is now allowed to actuate the motors. The other thing we have to take care of, is the Gazebo part. So we need to start Gazebo and spawn our model of the BlueROV.

Arm The Vehicle
***************
The following code is for most parts identical to the one in the previous :ref:`next_steps/ros:ROS` section. The :code:`arm_vehicle()` method now ensures, that the vehicle will get armed.

.. code-block:: python
   :linenos:
   
   #!/usr/bin/env python
   import rospy
   import math
   from mavros_msgs.msg import MotorSetpoint
   from mavros_msgs.srv import CommandBool


   class MyFirstNode():
      def __init__(self):
         rospy.init_node("motor_command_sender")
         self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",
                                             MotorSetpoint,
                                             queue_size=1)
         self.arm_vehicle()

      def arm_vehicle(self):
         # wait until the arming serivce becomes available
         rospy.wait_for_service("mavros/cmd/arming")
         # connect to the service
         arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
         # call the service to arm the vehicle until service call was successfull
         while not arm(True).success:
               rospy.logwarn("Could not arm vehicle. Keep trying.")
               rospy.sleep(1.0)
         rospy.loginfo("Armed successfully.")

      def run(self):
         rate = rospy.Rate(30.0)

         while not rospy.is_shutdown():
               msg = MotorSetpoint()
               msg.header.stamp = rospy.Time.now()
               # since the bluerov has 8 motors, the setpoint list holds 8 values
               t = rospy.get_time()
               msg.setpoint[0] = 0
               msg.setpoint[1] = 0
               msg.setpoint[2] = 0
               msg.setpoint[3] = 0
               msg.setpoint[4] = 0.2 * math.sin(t)
               msg.setpoint[5] = -0.2 * math.sin(t)
               msg.setpoint[6] = -0.2 * math.sin(t)
               msg.setpoint[7] = 0.2 * math.sin(t)

               self.setpoint_pub.publish(msg)

               rate.sleep()


   def main():
      node = MyFirstNode()
      node.run()


   if __name__ == "__main__":
      main()



Create A Launch Setup
*********************

Create a new launchfile. You could name it :file:`first_simulation.launch` for example:

.. image:: /res/images/create_launchfile.gif

The launchfile has to start the :file:`motor_command_sender` node, the Gazebo simulation and spawns the BlueROV.

It could look like this:

.. code-block:: xml
   :linenos:

   <launch>
      <!-- launch the motor_command_sender node-->
      <node name="motor_command_sender" pkg="awesome_package" type="motor_command_sender.py"/>

      <!-- start the gazebo simulator and an empty world -->
      <include file="$(find bluerov_sim)/launch/gazebo_base.launch" />
      
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
   </launch>

To start a the setup run:

.. code-block:: sh

   roslaunch awesome_package first_simulation.launch

The result should look similar to:

.. image:: /res/images/gazebo_awesome_package.gif

Get Sensor Data
===============

At this point we know the basics of actuating the vehicle. But to know how we want to actuate the vehicle, we might depend on some sensor input. 

The BlueROV has a pressure sensor. The output of the pressure sensor is published under the :file:`pressure` topic.

Theoretically we could use the :file:`motor_command_sender.py` and modify it's code to subscribe to the :file:`pressure` topic. But to keep things modular and separated, we add a new node to the :file:`awesome_package`. Let's name it :file:`depth_calculator.py`. 

.. note:: Keep in mind, you have to make every node executable! See :ref:`next_steps/ros:Write A Node`.

The source code might look like this:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy
   from sensor_msgs.msg import FluidPressure
   from std_msgs.msg import Float32


   def pressure_callback(pressure_msg, publisher):
      pascal_per_meter = 1.0e4
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

   rostopic echo /depth

