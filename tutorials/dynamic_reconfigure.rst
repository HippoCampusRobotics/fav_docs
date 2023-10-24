Dynamic Reconfigure
###################

.. role:: strike
   :class: strike

:strike:`Change parameters on the fly during runtime with the help of dynamic_reconfigure and never ever restart your whole setup to tune your hardcoded parameters.`
We left ``dynamic_reconfigure`` behind, when we migrated to ROS2.
ROS2 has built-in support for reconfigurable parameters.

Prerequisites
=============

* a ROS package for which you want to use :code:`dynamic_reconfigure` 

.. attention::

   In this example this package will be called :file:`awesome_package`. Replace occurences of this package name with your own package name. Do **not** copy-paste.

.. seealso::

   You are not quite sure what the necessary steps to create a new package are?
   Go back to :ref:`tutorials/ros_package:ROS Package`. 

Scenario
========

Imagine we have something like a PID-Controller (yeah, we thought it would be nice to have very *applied* example, i.e. it is very close to what you want to achieve yourself during this course) and we want to change the gains and maybe some other parameters as well while the node is running.
Things we would like to configure could be:

* PID gains
* limits for the integral term (there are many more anti-windup strategies. Use your preferred search engine for some inspiration.)
* Activate/Deactivate the integrator so the integrator is not accumulating control errors while the controller is not active.



Preparation
===========

We create a PID controller node called ``my_controller.py``.

Make it exectubale.

.. code-block:: sh

   chmod +x ~/fav/ros2/src/awesome_package/nodes/my_controller.py

Add the node the list of nodes to install in the :file:`CMakeLists.txt`.

.. code-block:: sh
   :linenos:
   :emphasize-lines: 4

   install(PROGRAMS
     ...
     nodes/some_other_node_that_might_also_be_in_this_list.py
     nodes/setpoint_publisher.py
     ...
     DESTINATION lib/${PROJECT_NAME}
   )

Build the workspace. Keep in mind this is only necessary because we changed something in :file:`CMakeLists.txt`.
No rebuilt is required if we only modify an already existing node.
Python code does not get compiled.

.. code-block:: sh

   build_ros

.. seealso::

   See :ref:`tutorials/ros_package:Run A Node` for more elaborate instructions for creating a node.

Write the Code
==============

Boilerplate
***********

Starting from a basic node setup:

.. code-block:: python
   :caption: my_controller.py
   :linenos:

   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node


   class MyControlNode(Node):

       def __init__(self):
           super().__init__(node_name='my_controller')


   def main():
       rclpy.init()
       node = MyControlNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass


   if __name__ == '__main__':
       main()

.. note::

   Wondering what the ``try`` statement is good for?
   When hitting :kbd:`Ctrl` + :kbd:`C`, a keyboard interrupt is triggered.
   If unhandled this will cause annoying exceptions to be printed in the terminal, even though nothing bad happened.
   Hence the ``except KeyboardInterrupt`` will catch this exceptions.
   The ``pass`` keyboard instructs the python interpreter to do nothing.

In this example we won't implement a PID controller.
The node will only store the configuration parameters in variables and print them for demonstration purpose. 

Declare Parameters
******************

So let's start with the parameters.
We declare them but do not set default values.
That is a choice we are making.
This way we can make sure, that the values have to be provided when starting the node.

.. code-block:: python
   :linenos:
   :emphasize-lines: 12-35

   #!/usr/bin/env python3

   import rclpy
   from hippo_msgs.msg import Float64Stamped
   from rclpy.node import Node


   class MyControlNode(Node):

       def __init__(self):
           super().__init__(node_name='my_controller')
           self.init_params()

       def init_params(self):
           self.declare_parameters(
               namespace='',
               parameters=[
                   ('gains.p', rclpy.Parameter.Type.DOUBLE),
                   ('gains.i', rclpy.Parameter.Type.DOUBLE),
                   ('gains.d', rclpy.Parameter.Type.DOUBLE),
               ],
           )
           # the calls to get_parameter will raise an exception if the paramter
           # value is not provided when the node is started.
           param = self.get_parameter('gains.p')
           self.get_logger().info(f'{param.name}={param.value}')
           self.p_gain = param.value

           param = self.get_parameter('gains.i')
           self.get_logger().info(f'{param.name}={param.value}')
           self.i_gain = param.value

           param = self.get_parameter('gains.d')
           self.get_logger().info(f'{param.name}={param.value}')
           self.d_gain = param.value


   def main():
       rclpy.init()
       node = MyControlNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass


   if __name__ == '__main__':
       main()

If we now run the node with

.. code-block:: sh

   ros2 run awesome_package my_controller.py

an exception will be raised.

.. code-block:: sh

   rclpy.exceptions.ParameterUninitializedException: The parameter 'gains.p' is not initialized
   [ros2run]: Process exited with failure 1

This was to be expected.
To test our node before we integrate it in a launch file setup, we can pass the arguments in the terminal with ``--ros-args`` and `-p`.

.. code-block:: sh

   ros2 run awesome_package my_controller.py --ros-args -p gains.p:=1.0 -p gains.i:=0.01 -p gains.d:=0.0

No exception is raised! An the logs reflect our manually set parameter values |partying_face|.

.. note::

   We will set these parameters in a more convenient way later on when we create a launch file setup.

We can see the parameters while the node is running

.. code-block:: sh

   ros2 param list

yielding

.. code-block:: sh
   :emphasize-lines: 3-5

   ❯ ros2 param list
   /my_controller:
     gains.d
     gains.i
     gains.p
     start_type_description_service
     use_sim_time

Get a parameter value

.. code-block:: sh
   
   ❯ ros2 param get /my_controller gains.p
   Double value is: 1.0

.. hint::

   For almost any command you can run ``my_command --help`` to get information on how to use the command.
   This is also true for ``ros2 param``, ``ros2 param get``, ``ros2 param set``, etc.

   For example

   .. code-block:: sh

      ros2 param get --help

   Use this kind of help.
   This way you can figure out yourself how to use all these tools without relying on other people's tutorials.

We can also **set** the parameter value, for example

.. code-block:: sh

   ros2 param set /my_controller gains.p 3.0

But currently our node does not handle these changes of the parameters.

Handle Parameter Changes
************************

.. code-block:: python
   :linenos:
   :emphasize-lines: 6,38,40-52

   #!/usr/bin/env python3

   import rclpy
   from hippo_msgs.msg import Float64Stamped
   from rclpy.node import Node
   from rcl_interfaces.msg import SetParametersResult


   class MyControlNode(Node):

       def __init__(self):
           super().__init__(node_name='my_controller')
           self.init_params()

       def init_params(self):
           self.declare_parameters(
               namespace='',
               parameters=[
                   ('gains.p', rclpy.Parameter.Type.DOUBLE),
                   ('gains.i', rclpy.Parameter.Type.DOUBLE),
                   ('gains.d', rclpy.Parameter.Type.DOUBLE),
               ],
           )
           # the calls to get_parameter will raise an exception if the paramter
           # value is not provided when the node is started.
           param = self.get_parameter('gains.p')
           self.get_logger().info(f'{param.name}={param.value}')
           self.p_gain = param.value

           param = self.get_parameter('gains.i')
           self.get_logger().info(f'{param.name}={param.value}')
           self.i_gain = param.value

           param = self.get_parameter('gains.d')
           self.get_logger().info(f'{param.name}={param.value}')
           self.d_gain = param.value

           self.add_on_set_parameters_callback(self.on_params_changed)

       def on_params_changed(self, params):
           param: rclpy.Parameter
           for param in params:
               self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
               if param.name == 'gains.p':
                   self.p_gain = param.value
               elif param.name == 'gains.i':
                   self.i_gain = param.value
               elif param.name == 'gains.d':
                   self.d_gain = param.value
               else:
                   continue
           return SetParametersResult(succesful=True, reason='Parameter set')


   def main():
       rclpy.init()
       node = MyControlNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass


   if __name__ == '__main__':
       main()

Try setting different values for the declared parameters with ``ros2 param set`` as explained above.
Verify that the logs of the node should reflect the parameter changes.

Use Paramter Files
==================

.. todo::

   Yeah, that's still missing.
   Will be updated soon!
   The impatient ones of you do not have to wait for that, of course!
   We think you could probably also figure out how to do this with quick internet research.

Command Line
************

Launch File
***********

