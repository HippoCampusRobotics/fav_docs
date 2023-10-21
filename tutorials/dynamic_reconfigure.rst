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

   In this example this package will be called :file:`toy_example`. Replace occurences of this package name with your own package name. Do **not** copy-paste.

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

   chmod +x ~/fav/ros2/src/toy_example/nodes/my_controller.py

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

.. todo::

   The following text needs an overhaul for ROS2

.. code-block:: python
   :caption: my_controller.py
   :linenos:

   #!/usr/bin/env python
   import rospy
   from dynamic_reconfigure.server import Server
   from toy_example.cfg import PidControlConfig
   import threading
   
   
   class MyControlNode():
       def __init__(self):
           rospy.init_node("my_controller")
           self.data_lock = threading.RLock()
           # the assigned values do not matter. They get overwritten by
           # dynamic_reconfigure as soon as the dynamic_reconfigure server is
           # created.
           self.p_gain = 0.0
           self.i_gain = 0.0
           self.d_gain = 0.0
           self.integrator_active = False
           self.integral_lower_limit = 0.0
           self.integral_upper_limit = 0.0
   
           self.dyn_server = Server(PidControlConfig, self.on_pid_dyn_reconfigure)
   
       def on_pid_dyn_reconfigure(self, config, level):
           # the config parameters are provided as dictionary. The keys are the
           # parameter names we specified in cfg/PidControl.cfg
   
           # use data_lock to avoid parallel modifications of the variables
           # from different threads (here the main thread running the loop in the
           # run() method and the thread runing the dynamic_reconfigure callback).
           with self.data_lock:
               self.p_gain = config["p_gain"]
               self.i_gain = config["i_gain"]
               self.d_gain = config["d_gain"]
               self.integral_lower_limit = config["integral_lower_limit"]
               self.integral_upper_limit = config["integral_upper_limit"]
               self.integrator_active = config["integrator_active"]
           return config
   
       def run(self):
           r = rospy.Rate(1)
           while not rospy.is_shutdown():
               # use data_lock to avoid parallel modifications of the variables
               # from different threads (here the main thread running this loop
               # and the thread runing the dynamic_reconfigure callback)
               with self.data_lock:
                   print("p_gain: {}\ni_gain: {}\nd_gain: {}"
                         "\nintegral_lower_limit: {}\nintegral_upper_limit: {}"
                         "\nintegrator_active: {}\n---".format(
                             self.p_gain, self.i_gain, self.d_gain,
                             self.integral_lower_limit, self.integral_upper_limit,
                             self.integrator_active))
               r.sleep()
   
   
   def main():
       node = MyControlNode()
       node.run()
   
   
   if __name__ == "__main__":
       main()
   
Dynamically Reconfigure a Node
******************************

Launch the ROS master

.. code-block:: sh

   roscore

run the node in another terminal

.. code-block:: sh
   
   rosrun toy_example my_controller.py

You will get an output like this:

.. code-block:: sh

   p_gain: 1.0
   i_gain: 0.0
   d_gain: 0.0
   integral_lower_limit: -0.2
   integral_upper_limit: 0.2
   integrator_active: False
   --- 

run :code:`rqt` in another terminal and open the :file:`plugins/configuration/dynamic_reconfigure` plugin.

.. image:: /res/images/dyn_rqt_plugin_open.png

If our node is running but the :code:`dynamic_reconfigure` plugin does not show it, click refresh. Select the node you want to reconfigure on the left side. The :code:`dynamic_reconfigure` options for all selected nodes will be shown. In our case we only have the :code:`my_controller` node. 

.. image:: /res/images/dyn_rqt_plugin_usage.png

We can change values by moving the sliders or entering them directly in the corresponding boxes (changing values with infinite limits via sliders is a bad idea).

If we look at the terminal output of our node, we should see changes in the printed values reflecting the changes we do in the :code:`dynamic_reconfigure` plugin.




