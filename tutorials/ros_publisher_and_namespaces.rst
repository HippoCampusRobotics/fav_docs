ROS Publisher and Namespaces
====================


Namespaces
**********

The concept of names and namespaces is explained in detail in the `ROS Documentation <https://design.ros2.org/articles/topic_and_service_names.html>`__. 

You can start nodes in namespaces (you can also have nested namespaces).
This means that the namespace gets prepended to all relative and private topic/action/service names.
We already used this in the :ref:`ROS Launch and Setup <tutorials/ros_launch_setup:ROS Launch Setup>`.
Every node (and also every node in included launch files) inside a ``GroupAction`` with a ``PushRosNamespace`` action is launched inside a namespace.

Write a Publisher
*******************

The typical workflow for writing a publisher should consist of:

#. What kind of message do you want to publish? There are several different types within the ROS-verse, each of which fulfills a different purpose. You can look them up in the searchengine of your choice 😉. Some of the most common ones are: ``std_msgs``, ``geometry_msgs``, ``control_msgs`` or ``sensor_msgs``. 

#. Initialize the publisher

#. Write a function to publish to the topic

#. Fill the message with sensible content


.. tab-set::

    .. tab-item:: 1. Choose Message Type

      Remember how in the tutorial :ref:`tutorials/ros_package:write a node` we created a node publishing actuator Setpoints in a sine wave manner? In the following expample, we want to publish the pose setpoint of the robot. For simplicity we only want to publish constant values. Of course, ROS already provides you with the perfect message type for that. ``PoseStamped`` is part of the ``geometry_msgs`` library. You can see the exact composition here `ROS Docs <https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html>`__.

    .. tab-item:: 2. Initialize the Message

      Before you can use a certain messgae type, make sure to import the corresponding library (See highlighted line in code below).
      The first actual step then is, to call the ``create_publisher()`` function from ``rclpy``. Usually this function takes at least three arguments: 

      #. The message type: ``msg_type``
      #. The topic name: ``topic``
      #. The queue size: ``qos_profile``. A good default value here is 1
      
      You have to **specify the topic name**, so that it is in a human readable format and so, that nodes can publish to and subscribe from the very same topic with a unique name. You can do this in three different ways:

      .. dropdown:: **Global**
         :color: primary

         .. code-block:: python

            self.pose_pub = self.create_publisher(PoseStamped, "/my_robot/pose_setpoint", 1)

         A topic name with a leading ``/`` will be resolved globally.
         This means that it does not matter if the node was launched in a namespace or not.
         The resulting topic name will be exactly ``/my_robot/pose_setpoint``.

         .. hint::
            Usually, we do not want to use global names.
            **Avoid them** if you do not have a good reason to use them.

      .. dropdown:: **Relative** (most common)
         :color: primary

         .. code-block:: python

            self.pose_pub = self.create_publisher(PoseStamped, "pose_setpoint", 1)

         A topic without leading ``/`` will be relative.
         This means that if the node was launched in a namespace, the namespace will get prepended.
         So, for example if the node was launched in the namespace ``my_robot``, the resolved topic name will become ``/my_robot/pose_setpoint``.
         In case the node was not launched inside any namespace, nothing will get prependended to the topic name.
         It will be just :file:`/pose_setpoint`.
         If the node is pushed into multiple namespaces, they all get prepended to the resolved topic name.

         .. hint::
            This will probably the **most common** way of specifying the topic name during this class.

      .. dropdown:: **Private**
         :color: primary

            .. code-block:: python

               self.debug_pub = self.create_publisher(ControlDebugMessage, '~/debug', 1)

         Private topics are similar to relative ones.
         The topic name starts with :file:`~`.
         The namespace will get prepended (if one has been specified), just as it will for relative topics.
         Additionally, the name of the node will *also* be prepended.
         So, if a node with the name ``pose_setpoints_pub`` has been started in the namespace ``my_robot``, the resolved topic name will be ``/my_robot/pose_setpoints_pub/debug``.
         Without a namespace, it would be :file:`/pose_setpoints_pub/debug`.

         .. hint::
            This way of specifying topic names is **less common**, but it is especially useful for debug messages and other messages that are closely tied to a specific node.
      
      .. code-block:: python
         :caption: pose_setpoints_pub.py
         :emphasize-lines: 5, 14


         #!/usr/bin/env python3

         import rclpy
         from rclpy.node import Node
         from geometry_msgs.msg import PoseStamped

         from tf_transformations import quaternion_from_euler
         import numpy as np

         class MyPoseSetpoints(Node):

            def __init__(self):
               super().__init__(node_name='pose_setpoints_pub')
               self.pose_pub = self.create_publisher(PoseStamped, "pose_setpoint", 1)

               # Create a timer. We use it to call a function with a defined rate.
               self.timer = self.create_timer(1 / 50, self.on_timer)

            def on_timer(self):
            self.publish_setpoint()

            def get_setpoint(self):
               x = 1.0
               y = 1.0
               z = 1.0
               roll = 0.0
               pitch = 0.0
               yaw = np.pi / 2.0

               qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

               return x, y, z, qx, qy, qz, qw

            def publish_setpoint(self):
               msg = PoseStamped()
               now = self.get_clock().now()
               msg.header.stamp = now.to_msg()

               x, y, z, qx, qy, qz, qw = self.get_setpoint()

               msg.pose.position.x = x
               msg.pose.position.y = y
               msg.pose.position.z = z

               msg.pose.orientation.x = qx
               msg.pose.orientation.y = qy
               msg.pose.orientation.z = qz
               msg.pose.orientation.w = qw

               self.pose_pub.publish(msg)

         def main():
         rclpy.init()
         node = MyPoseSetpoints()
         rclpy.spin(node)
         
         if __name__ == '__main__':
         main()

    .. tab-item:: 3. Create Publish Function

      Next, we have to add a method we can call from anywhere within the class to publish content to the topic we just created. In order to do that we have to take a look at the `exact definition <https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html>`__ of the message type again:

      .. image:: /res/images/message_type_screencast.gif

      Just like shown in the screencast, we can see exactly how the message is composed and write our code accordingly:

      .. code-block:: python
         :caption: pose_setpoints_pub.py
         :emphasize-lines: 34-50

         #!/usr/bin/env python3

         import rclpy
         from rclpy.node import Node
         from geometry_msgs.msg import PoseStamped

         from tf_transformations import quaternion_from_euler
         import numpy as np

         class MyPoseSetpoints(Node):

            def __init__(self):
               super().__init__(node_name='pose_setpoints_pub')
               self.pose_pub = self.create_publisher(PoseStamped, "pose_setpoint", 1)

               # Create a timer. We use it to call a function with a defined rate.
               self.timer = self.create_timer(1 / 50, self.on_timer)

            def on_timer(self):
            self.publish_setpoint()

            def get_setpoint(self):
               x = 1.0
               y = 1.0
               z = 1.0
               roll = 0.0
               pitch = 0.0
               yaw = np.pi / 2.0

               qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

               return x, y, z, qx, qy, qz, qw

            def publish_setpoint(self):
               msg = PoseStamped()
               now = self.get_clock().now()
               msg.header.stamp = now.to_msg()

               x, y, z, qx, qy, qz, qw = self.get_setpoint()

               msg.pose.position.x = x
               msg.pose.position.y = y
               msg.pose.position.z = z

               msg.pose.orientation.x = qx
               msg.pose.orientation.y = qy
               msg.pose.orientation.z = qz
               msg.pose.orientation.w = qw

               self.pose_pub.publish(msg)

         def main():
         rclpy.init()
         node = MyPoseSetpoints()
         rclpy.spin(node)
         
         if __name__ == '__main__':
         main()

    .. tab-item:: 4. Fill the Message

      Now all that is left is to assign, what we want to publish to the topic we just created. Let's say we want our robot to be at the euler coordinates (1, 1, 1), have both zero pitch and roll and a yaw of pi/2. Additionally, we have to consider, that ``PoseStamped`` works with qutaternions, so we need to convert our setpoint values:

      .. code-block:: python
         :caption: pose_setpoints_pub.py
         :emphasize-lines: 22-32

         #!/usr/bin/env python3

         import rclpy
         from rclpy.node import Node
         from geometry_msgs.msg import PoseStamped

         from tf_transformations import quaternion_from_euler
         import numpy as np

         class MyPoseSetpoints(Node):

            def __init__(self):
               super().__init__(node_name='pose_setpoints_pub')
               self.pose_pub = self.create_publisher(PoseStamped, "pose_setpoint", 1)

               # Create a timer. We use it to call a function with a defined rate.
               self.timer = self.create_timer(1 / 50, self.on_timer)

            def on_timer(self):
            self.publish_setpoint()

            def get_setpoint(self):
               x = 1.0
               y = 1.0
               z = 1.0
               roll = 0.0
               pitch = 0.0
               yaw = np.pi / 2.0

               qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

               return x, y, z, qx, qy, qz, qw

            def publish_setpoint(self):
               msg = PoseStamped()
               now = self.get_clock().now()
               msg.header.stamp = now.to_msg()

               x, y, z, qx, qy, qz, qw = self.get_setpoint()

               msg.pose.position.x = x
               msg.pose.position.y = y
               msg.pose.position.z = z

               msg.pose.orientation.x = qx
               msg.pose.orientation.y = qy
               msg.pose.orientation.z = qz
               msg.pose.orientation.w = qw

               self.pose_pub.publish(msg)

         def main():
         rclpy.init()
         node = MyPoseSetpoints()
         rclpy.spin(node)
         
         if __name__ == '__main__':
         main()

      .. note:: 
         For those of you, who haven't heard about quaternions yet and/or just want to get their mind blown 🤯. Here is a cool  `Interactive video <https://www.youtube.com/watch?v=d4EgbgTm0Bg&ab_channel=3Blue1Brown>`__ from 3Blue1Brown.
      
      Et voilà, we have written our first publisher |partying_face|

BlueROV
*******

You will only work with a single robot in the labs.
So, in the labs we will start everything in the :file:`bluerov01` namespace.
**But**, when we are working with the digital mock up in the simulation, we use a different namespace, which is :file:`bluerov00`.

.. note::
   If you have a controller subscribing to a setpoint topic, it might be a good idea to use a private name ``'~/setpoint'``.
   This way you avoid topic name conflicts in case you have another controller also subscribing to a setpoint topic.
