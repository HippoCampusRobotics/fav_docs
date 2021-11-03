Dynamic Reconfigure
===================

Change parameters on the fly during runtime with the help of :code:`dynamic_reconfigure` and never ever restart your whole setup to tune your hardcoded parameters |partying_face| . You can find the API documentation and another tutorial guide in the `ROS wiki <http://wiki.ros.org/dynamic_reconfigure>`__. 

Prerequisites
*************

* a ROS package for which you want to use :code:`dynamic_reconfigure` 

.. attention:: In this example this package will be called :file:`toy_example`. Replace occurences of this package name with your own package name. Do **not** copy-paste.

Scenario
********

Imagine we have something like a PID-Controller and we want to change the gains and maybe some other parameters as well while the node is running. Things we would like to configure could be:

* PID gains
* limits for the integral term (there are many more anti-windup strategies. Use your preferred search engine for some inspiration.)
* Activate/Deactivate the integrator so the integrator is not accumulating control errors while the controller is not active.

Config File
***********

Config files define the parameters we use for a :code:`dynamic_reconfigure` service. In the following sections we will create such a config file and add some parameters.

Create a Config File
####################

#. Create a directory called :file:`cfg` for your package.

   .. tabs::

      .. group-tab:: GUI

         .. image:: /res/images/dyn_create_cfg_dir.gif

      .. code-tab:: sh Terminal

         cd ~/fav/catkin_ws/src/toy_example && mkdir cfg

#. Create a config file with a meaningful name.
   
   .. tabs::

      .. group-tab:: GUI

         .. image:: /res/images/dyn_create_cfg_file.gif
      
      .. code-tab:: sh Terminal

         touch PidControl.cfg

Config File Boilerplate
#######################

Replace :code:`"toy_example"` with the name of your package.

.. code-block:: python
   :caption: PidControl.cfg
   :linenos:

   #!/usr/bin/env python
   PACKAGE = "toy_example"

   from dynamic_reconfigure.parameter_generator_catkin import *

   gen = ParameterGenerator()

   # here wee need to add our reconfigure parameters

   exit(gen.generate(PACKAGE, "my_control_node", "PidControl"))

:code:`"my_control_node"` is the name of a node that could use this :code:`dynamic_reconfigure` configuration. It is used for docs generation only and can be whatever we like. The last parameter :code:`"PidControl"` determines the name of the generated Python or C++ code. This will be relevant when we import the generated python file in our node. :code:`Config.py` will always be appended. So in our case the generated python file name will be :file:`PidControlConfig.py`

Add Parameters
##############

We can use different parameter types:

* :code:`int_t`
* :code:`double_t`
* :code:`str_t`
* :code:`bool_t`

Even :code:`enums` are possible but we will leave this out for now. For the PID gains and for the integral limits we use :code:`double_t`. For activating/deactivating the integrator of the PID control we use :code:`bool_t`.

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   PACKAGE = "toy_example"

   from dynamic_reconfigure.parameter_generator_catkin import *

   gen = ParameterGenerator()

   # here wee need to add our reconfigure parameters
   gen.add(name="p_gain", paramtype=double_t, level=0, description="Proportional gain", default=1.0, min=None, max=None)
   gen.add(name="i_gain", paramtype=double_t, level=0, description="Integral gain.", default=0, min=None, max=None)
   gen.add(name="d_gain", paramtype=double_t, level=0, description="Derivative gain.", default=0, min=None, max=None)
   gen.add(name="integral_lower_limit", paramtype=double_t, level=0, description="Integral lower limit.", default=-0.2, min=-1.0, max=1.0)
   gen.add(name="integral_upper_limit", paramtype=double_t, level=0, description="Integral upper limit.", default=0.2, min=-1.0, max=1.0)
   gen.add(name="integrator_active", paramtype=bool_t, level=0, description="Activate or deactivate the integrator.", default=False)

   exit(gen.generate(PACKAGE, "toy_example", "PidControl.cfg"))



The values for :code:`min` and :code:`max` are optional. If you do not want to/cannot specify them, set them to :code:`None` or omit them completely.

.. note:: The parameter name must be a valid identifier. Do not use spaces or leading numbers.

Add Config File to CMakeLists.txt
#################################

We have to modify :file:`CMakeLists.txt` to tell catkin to build our :code:`dynamic_reconfigure` configuration. Find :code:`generate_dynamic_reconfigure_options()` and uncomment it or just add it manually.

.. image:: /res/images/dyn_cmakelists.gif

Rebuild the Workspace
#####################

Build

.. code-block:: sh

   cd ~/fav/catkin_ws && catkin build

Resource :file:`~/.bashrc`

.. code-block:: sh

   source ~/.bashrc

Writing a Reconfigurable Node
*****************************

Preparation
###########

Create the python file.

.. tabs::

   .. group-tab:: GUI

      .. image:: /res/images/dyn_create_node.gif
      
   .. code-tab:: sh Terminal

      cd ~/fav/catkin_ws/src/toy_example
      mkdir nodes
      touch my_controller.py


Make it exectubale.

.. code-block:: sh

   chmod +x ~/fav/catkin_ws/src/toy_example/nodes/my_controller.py

Add the node to :code:`catkin_install_python()`.

.. image:: /res/images/dyn_install_python.gif

Build the workspace

.. code-block:: sh

   cd ~/fav/catkin_ws && catkin build

Write the Code
##############

Starting from a basic node setup:

.. code-block:: python
   :caption: my_controller.py
   :linenos:

   #!/usr/bin/env python
   import rospy
   
   
   class MyControlNode():
       def __init__(self):
           rospy.init_node("my_controller")
   
       def run(self):
           r = rospy.Rate(1)
           while not rospy.is_shutdown():
               r.sleep()
   
   
   def main():
       node = MyControlNode()
       node.run()
   
   
   if __name__ == "__main__":
       main()
   
We will import the :code:`dynamic_reconfigure` server and our :code:`PidControlConfig` we created before. Similar to writing a :code:`rospy.Subscriber` we set a callback for :code:`dynamic_reconfigure`. Each time the configuration gets changed the callback will be executed.

In this example we won't implement a PID controller. The node will only store the configuration parameters in variables and print them for demonstration purpose. 

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

You will get an output like this:

.. code-block:: sh

   p_gain: 1.0
   i_gain: 0.0
   d_gain: 0.0
   integral_lower_limit: -0.2
   integral_upper_limit: 0.2
   integrator_active: False
   ---

.. code-block:: sh

   rosrun toy_example my_controller.py

run :code:`rqt` in another terminal and open the :file:`plugins/configuration/dynamic_reconfigure` plugin.

.. image:: /res/images/dyn_rqt_plugin_open.png

If our node is running but the :code:`dynamic_reconfigure` plugin does not show it, click refresh. Select the node you want to reconfigure on the left side. The :code:`dynamic_reconfigure` options for all selected nodes will be shown. In our case we only have the :code:`my_controller` node. 

.. image:: /res/images/dyn_rqt_plugin_usage.png

We can change values by moving the sliders or entering them directly in the corresponding boxes (changing values with infinite limits via sliders is a bad idea).

If we look at the terminal output of our node, we should see changes in the printed values reflecting the changes we do in the :code:`dynamic_reconfigure` plugin.




