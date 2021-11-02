ROS
###

In this section you will:

* create a new Catkin package
* write a node in Python
* run a node via :code:`rosrun`

.. attention::

   The following tutorial **is not meant as a step-by-step solution for the first assignment.** These are just **toy examples** to demonstrate how to use ROS and interact with the simulated BlueROV *in an easy to follow manner*. Therefore, we do not claim that these code snippets are complete and we use some funny names at times. Please do **not** copy-paste them.



Create A Package
================

If you have completed all the installation instructions in :ref:`installation/install_ros:Install ROS`, you have already created a Catkin workspace at :file:`~/fav/catkin_ws`. The workspace contains several directories::

   ~/fav/catkin_ws
   ├── build
   ├── devel
   ├── logs
   └── src


Probably the only one you will be working with is the :file:`src` directory. This is the place where the so called packages are. 

For ROS (actually it should be catkin and not ROS, since catkin is the build tool. This is also the reason why we call it catkin workspace and not ROS workspace. But in the end it does not matter if we are somewhat unprecise) to recognize a directory as package it requires two files:

* :file:`package.xml`
* :file:`CMakeLists.txt`

Otherwise a package is nothing but a normal directory containing arbitrary files and subdirectories.

For this guide it is not necessary to go into details too much, but if you like to know more about packages, you can read the article about packages in the `ROS Wiki <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>`_.

There are two ways to create packages in a convenient way. By convenient I mean without worrying about :file:`package.xml` and :file:`CMakeLists.txt` too much. 

Create A Package With VS Code
*****************************

Open :file:`~/fav/catkin_ws` with VS Code. Press :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`P` and enter :code:`create catkin package`:

.. image:: /res/images/vscode_new_catkin_package.png

You will be asked to enter a name and dependencies (I think "awesome_package" is quite a nice name for an example package). You can skip the dependencies for now and leave the field empty.

.. seealso:: For more details on dependencies see the `ROS Wiki <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>`_.

Create A Package In The Commandline
***********************************

Since Catkin expects packages to be in the :file:`src` directory, we have to make sure, we are currently there:

.. code-block:: sh

   cd ~/fav/catkin_ws/src

.. code-block:: sh

   catkin create pkg awesome_package --catkin-deps rospy roscpp std_msgs geometry_msgs sensor_msgs

That's it. You have just created your first catkin package.

If you have (and you should) opened your Catkin workspace in VS Code, your workspace could look like in the following image (for the screenshot the newly created package was named :file:`pressure_example`):

.. image:: /res/images/vscode_catkin_overview.png

Write A Node
============

In general, you have the choice to write nodes either in Python or in C++. For the sake of simplicity we recommend Python. If you haven't already worked with one of these languages, in some regards Python might feel similiar to Matlab.

Before we can write a node, we create a :file:`nodes` directory to keep things neat and clean. It is not strictly required (ROS will find your node as long as it is in your package, no matter in which subdirectory it is), but it complies with conventions.

Right click :file:`awesome_package` and choose **New Folder** and name it :file:`nodes`. Right click :file:`nodes` and choose **New File**. Name it :file:`motor_command_sender.py`. It should open automatically.

.. image:: /res/images/vscode_create_node.gif

We have to make the Python file executable. To do so, enter the following command in your terminal (for example the integrated one in VS Code):

.. code-block:: sh

   chmod +x ~/fav/catkin_ws/src/awesome_package/nodes/motor_command_sender.py

The general syntax is :code:`chmod +x PATH_TO_THE_FILE`.

If you feel more comfortable with the graphical interface, you can also set the executable flag in the file browser:

.. image:: /res/images/executable.gif

.. note:: Each time you create a new node, make sure you have made it executable.

The first line of your node needs to be:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python

so your system knows your file should be executed as a Python file.

Your first node could look like:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy
   import math
   from mavros_msgs.msg import MotorSetpoint


   class MyFirstNode():
      def __init__(self):
         rospy.init_node("motor_command_sender")
         self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",
                                             MotorSetpoint,
                                             queue_size=1)

      def run(self):
         rate = rospy.Rate(30.0)

         while not rospy.is_shutdown():
               msg = MotorSetpoint()
               msg.header.stamp = rospy.Time.now()
               # since the bluerov has 8 motors, the setpoint list holds 8 values
               t = rospy.get_time()
               msg.setpoint[0] = 0.2 * math.sin(t)
               msg.setpoint[1] = -0.2 * math.sin(t)
               msg.setpoint[2] = 0.2 * math.cos(t)
               msg.setpoint[3] = -0.2 * math.cos(t)
               msg.setpoint[4] = 0.4 * math.sin(t)
               msg.setpoint[5] = -0.4 * math.sin(t)
               msg.setpoint[6] = 0.4 * math.cos(t)
               msg.setpoint[7] = -0.4 * math.cos(t)
               
               self.setpoint_pub.publish(msg)

               rate.sleep()


   def main():
      node = MyFirstNode()
      node.run()


   if __name__ == "__main__":
      main()

Run A Node
==========

.. attention:: For each node we have to modify the :file:`CMakeLists.txt` of the corresponding package. Add the node's path relative to the package's root to the :code:`catkin_install_python()` call.

For our first node we add the highlighted line to the :file:`CMakeLists.txt`.

.. code-block:: bash
   :emphasize-lines: 2

   catkin_install_python(PROGRAMS
     nodes/motor_command_sender.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

Make sure you UNcomment these lines -> remove the :code:`#` characeters. Every time you modify the :file:`CMakeLists.txt` rebuild your workspace with :code:`catkin build` and to be super save you might also want to resource your workspace setup with :code:`source ~/.bashrc`.


If you want to run a Python programm, normally you would use a command like :code:`python /path/to/your/file/python_file.py`. This would work for our node, too. But instead of running our node by entering :code:`python ~/fav/catkin_ws/src/awesome_package/nodes/motor_command_sender.py`, where we have to explicitly tell Python where it can find our file :file:`motor_command_sender.py`, we can use :code:`rosrun`. One of the advantages of :code:`rosrun` is that we do not have to know, where the program/node is that we want to run. 

The general usage of the :code:`rosrun` command is :code:`rosrun <package_name> <executable_name>`. So for our :file:`awesome_package` and its :file:`motor_command_sender.py` it would be:

.. code-block:: sh

   rosrun awesome_package motor_command_sender.py

If you try to do so right now, you will likely get the error message:

.. code-block:: sh

   [rospack] Error: package 'awesome_package' not found

We created a Catkin package, but we haven't told ROS of it yet. To do so, we rebuild our Catkin workspace:

.. code-block:: sh

   catkin build

.. note:: You have to execute :code:`catkin build` from within your Catkin workspace. So always make sure you are in the :file:`~/fav/catkin_ws` directory.

The paths of your packages get updated. To apply these updated paths, run:

.. code-block:: sh

   source ~/.bashrc

.. note:: Every time we create a new package we need to build our Catkin workspace with :code:`catkin build` and apply the updated package paths with :code:`source ~/.bashrc`.

Nodes also require the ROS Master to run. Open two terminals (for example by splitting VS Code's internal terminal). In the first one, start the ROS master with 

.. code-block:: sh

   roscore

In the second one, start the node via :code:`rosrun`:

.. code-block:: sh

   rosrun awesome_package motor_command_sender.py

.. image:: /res/images/rosrun_motor_command_sender.gif

.. hint:: Just to remind you: you stop running programs in a terminal by the shortcut :kbd:`Ctrl` + :kbd:`C`.

In the node's source code you can see, the motor commands, that are sent, are :code:`sin` and :code:`cos` signals.

We can use :code:`rqt-multiplot` to visualize the data. The following screenshot shows the motor commands for the first four motors.

.. image:: /res/images/rqt_multiplot.png