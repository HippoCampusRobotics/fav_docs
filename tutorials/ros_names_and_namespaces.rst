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
