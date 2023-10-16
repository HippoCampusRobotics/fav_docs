Names and Namespaces
====================


Namespaces
**********

The concept of names and namespaces is explained in detail in the `ROS Documentation <https://design.ros2.org/articles/topic_and_service_names.html>`__. 

You can start nodes in namespaces (you can also have nested namespaces).
This means that the namespace gets prepended to all relative and private topic/action/service names.
We already used this in the :ref:`launch file tutorial <tutorials/ros_launch_setup:ROS Launch Setup>`.
Every node (and also every node in included launch files) inside a ``GroupAction`` with a ``PushRosNamespace`` action is launched inside a namespace.

Names
*****

If you have a node subscribing or publishing to/from a topic, you have to specify the topic name. You can do this in three different ways:

Global
   .. code-block:: python

      self.pose_pub = self.create_publisher(PoseStamped, "/my_robot/pose", 1)
   
   A topic name with a leading ``/`` will be resolved globally.
   This means that it does not matter if the node was launched in a namespace or not.
   The resulting topic name will be exactly ``/my_robot/pose``.

   .. hint::
      Usually, we do not want to use global names.
      Avoid them if you do not have a good reason to use them.

Relative
   .. code-block:: python

      self.pose_pub = self.create_publisher(PoseStamped, "my_robot/pose", 1)

   A topic without leading ``/`` will be relative.
   This means that if the node was launched in a namespace the namespace will get prepended.
   So, for example if the node was launched in the namespace ``my_robot``, the resolved topic name will become ``/my_robot/position``.
   In case the node was not launched inside any namespace, nothing will get prependended to the topic name.
   It will be just :file:`/position`.
   If the node is pushed into multiple namespaces, they all get prepended to the resolved topic name.

   .. hint::
      This will probably the most common way of specifying the topic name during this class.


Private
   .. code-block:: python

      class MyNode(Node):
         
         def __init__(self):
            super().__init__('my_controller')
            self.debug_pub = self.create_publisher(ControlDebugMessage, '~/debug', 1)

   Private topics are similar to relative ones.
   The topic name start with :file:`~`.
   The namespace will get prepended if it has been specified, just as it is for relative topics.
   Additionally, the name of the node will *also* be prepended.
   So, if the node with the name ``my_controller`` has been started in the namespace ``my_robot``, the resolved topic name will be ``/my_robot/my_controller/debug``.
   Without a namespace it would be :file:`/my_controller/debug`.

   .. hint::
      This way of specifying topic names is less common, but it is especially useful for debug messages and other messages that are closely tied to a specific node.

BlueROV
*******

You will only work with a single robot.
Still, it is nice to have things clean and start everything at least in the :file:`bluerov00` namespace. 

.. note::
   If you have a controller subscribing to a setpoint topic, it might be a good idea to use a private name ``'~/setpoint'``.
   This way you avoid topic name conflicts in case you have another controller also subscribing to a setpoint topic.
