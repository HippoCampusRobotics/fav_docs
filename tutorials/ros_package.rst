ROS Package
###########

In this section you will:

* create a new ros package
* write a node in :code:`python`
* run a node via :code:`ros2 run`
* use the tools :code:`ros2 node info` and :code:`ros2 topic echo`

.. attention::

   The following tutorial **is not meant as a step-by-step solution for the first assignment.** These are just **toy examples** to demonstrate how to use ROS and interact with the simulated BlueROV *in an easy to follow manner*. Therefore, we do not claim that these code snippets are complete and we use some funny names at times. Please do **not** copy-paste them.



Create A Package
================

If you have completed all the installation instructions in :ref:`installation/install_ros:Install ROS`, you have already created a Catkin workspace at :file:`~/fav/ros2`.
The workspace contains several directories::

   ~/fav/ros2
   ├── build
   ├── install
   ├── log
   └── src


Probably the only one you will be working with is the :file:`src` directory. This is the place where the so called *packages* are. 

For our code directories to be recognized as packages, two files are required:

* :file:`package.xml`
* :file:`CMakeLists.txt`

Otherwise a package is nothing but a normal directory containing arbitrary files and subdirectories.
Packages can also be in arbitrary subdirectories, since the build system will look for directories containing a :file:`package.xml` recursively to identify packages.
An example for this is the :file:`hippo_core` repository you cloned during the setup instructions.
The :file:`hippo_core` directory itself is not a package, but contains packages as subdirectories like :file:`hippo_common` or :file:`hippo_msgs`.

For this guide it is not necessary to go into details too much.
But if you like to know more about packages, you can read the article about packages in the `ROS Docs <https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

.. hint:: In ROS2 there is the option to create native Python packages. For this class, we stick to the CMake way of organizing packages. Even if we write pure Python packages. So in this regard our instructions differ from the official documentation.

Go to the :file:`src` directory

.. code-block:: sh

   cd ~/fav/ros2/src

and create the package directory

.. code-block:: sh

   mkdir awesome_package

Remember, we need at least :file:`package.xml` and :file:`CMakeLists.txt`.
Almost minimal examples are presented in the following.
Take a look at the highlighted lines.
Replace the project's name with your own package name.

.. code-block:: cmake
   :linenos:
   :caption: CMakeLists.txt
   :emphasize-lines: 2

   cmake_minimum_required(VERSION 3.5)
   project(awesome_package)
   find_package(ament_cmake REQUIRED)
   find_package(ament_cmake_python REQUIRED)
   find_package(rclpy REQUIRED)

   install(PROGRAMS
     DESTINATION lib/${PROJECT_NAME}
   )

   ament_package()

.. code-block:: xml
   :linenos:
   :caption: package.xml
   :emphasize-lines: 4

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>awesome_package</name>
     <version>0.0.0</version>
     <description>Our super awesome package</description>

     <maintainer email="someones.mail.address@tuhh.de">Someones name</maintainer>

     <!-- One license tag required, multiple allowed, one license per tag -->
     <!-- Commonly used license strings: -->
     <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
     <license>GPLv2</license>

     <url type="website">hippocampusrobotics.github.io/docs</url>

     <author email="someones.mail@tuhh.de">Someones name</author>

     <buildtool_depend>ament_cmake</buildtool_depend>
     <buildtool_depend>ament_cmake_python</buildtool_depend>

     <depend>rclpy</depend>

     <!-- The export tag contains other, unspecified, tags -->
     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>

That's it. You have just created your first package.
Your package structure should look similar to::

   ~/fav/ros2/src
   └── awesome_package
       ├── CMakeLists.txt
       └── package.xml

We can now build our workspace

.. code-block:: sh

   build_ros

and source the newly created package.

.. code-block:: sh

   . ~/.bashrc

This only needs to be done once a new package is created.
Nothing bad happens if we are a bit overly cautios regarding sourcing our :file:`.bashrc`.
But it does nothing good either.
So we might want to save it up for the cases where it is actually required.

If the commands mentioned above completed without errors, we can check if our newly created is detected correctly.
The following command should give as the installation path of our package.

.. code-block:: sh

   ros2 pkg prefix awesome_package

In case things did not work out as expected, we might get :code:`Package not found` as response.
This indicates that we (most likely) messed something up while following the instructions above.
Double check everything and if this does not fix the problem ask your favorite research associate.

Write A Node
============

In general, you have the choice to write nodes either in Python or in C++.
For the sake of simplicity we recommend Python.
If you haven't already worked with one of these languages, in some regards Python might feel similiar to Matlab.

Before we can write a node, we create a :file:`nodes/` directory to keep things neat and clean.
It is not strictly required (ROS will find your node as long as it is in your package, no matter in which subdirectory it is), but it complies with conventions.

Right click :file:`awesome_package` and choose **New Folder** and name it :file:`nodes`. Right click :file:`nodes` and choose **New File**. Name it :file:`setpoint_publisher.py`. It should open automatically.

.. image:: /res/images/vscode_create_node.gif

We have to make the Python file executable.
To do so, enter the following command in your terminal (for example the integrated one in VS Code):

.. code-block:: sh

   chmod +x ~/fav/ros2/src/awesome_package/nodes/setpoint_publisher.py

.. hint:: Just in case the integrated terminal is not open: You can open it with :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`\``.

The general syntax is :code:`chmod +x PATH_TO_THE_FILE`.

If you feel more comfortable with the graphical interface, you can also set the executable flag in the file browser by right clicking it and open its properties:

.. image:: /res/images/executable.gif

.. note:: Each time you create a new node, make sure you have made it executable.

The first line of your node needs to be:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python3

so your system knows your file should be executed as a Python file.

Your first node could look like:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python3

   import math

   import rclpy
   from hippo_msgs.msg import ActuatorSetpoint
   from rclpy.node import Node


   class MyFirstNode(Node):

       # the __init__ function gets called, when we create the object, i.e.
       # run something like
       #
       # node = MyFirstNode()
       def __init__(self):
           # we initialize the rclpy Node with a unique name
           super().__init__(node_name='my_first_node')

           # create a publisher. we need to specify the message type and the topic
           # name. The last argument specifies the queue length
           self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                   'thrust_setpoint', 1)
           self.torque_pub = self.create_publisher(ActuatorSetpoint,
                                                   'torque_setpoint', 1)
           self.timer = self.create_timer(1 / 50, self.on_timer)

       def on_timer(self):
           self.publish_setpoints()

       def publish_setpoints(self):
           # create the message object
           thrust_msg = ActuatorSetpoint()
           now = self.get_clock().now()
           thrust_msg.header.stamp = now.to_msg()
           # get the time as floating point number in seconds
           t = now.nanoseconds * 1e-9

           thrust_msg.x = 0.5 * math.sin(t)
           thrust_msg.y = -0.5 * math.sin(t)
           thrust_msg.z = 0.5 * math.cos(t)

           torque_msg = ActuatorSetpoint()
           torque_msg.header.stamp = now.to_msg()

           torque_msg.x = 0.4 * math.sin(t)
           torque_msg.y = -0.4 * math.sin(t)
           torque_msg.z = 0.4 * math.cos(t)
           # publish the messages with the publishers we created during the object
           # initialization
           self.thrust_pub.publish(thrust_msg)
           self.torque_pub.publish(torque_msg)


   def main():
       rclpy.init()
       node = MyFirstNode()
       rclpy.spin(node)


   if __name__ == '__main__':
       main()


Run A Node
==========

.. attention:: For each node we have to modify the :file:`CMakeLists.txt` of the corresponding package. Add the node's path relative to the package's root to the :code:`install()` call.

For our first node we add the highlighted line to the :file:`CMakeLists.txt`.

.. code-block:: cmake
   :emphasize-lines: 2

   install(PROGRAMS
     nodes/setpoint_publisher.py
     DESTINATION lib/${PROJECT_NAME}
   )

**Every** time you modify the :file:`CMakeLists.txt` rebuild your workspace with :code:`build_ros` and to be super save you might also want to resource your workspace setup with :code:`. ~/.bashrc`.
The latter is only required if we added a new node.

If you want to run a Python program, normally you would use a command like :code:`python3 /path/to/your/file/python_file.py`.
This would work for our node, too.
But instead of running our node by entering :code:`python ~/fav/ros2/src/awesome_package/nodes/setpoint_publisher.py`, where we have to explicitly tell Python where it can find our file :file:`setpoint_publisher.py`, we can use :code:`ros2 run`. One of the advantages of :code:`ros2 run` is that we do not have to know where the program/node is that we want to run.
The command finds the source file on its own.

The general usage of the :code:`ros2 run` command is :code:`ros2 run <package_name> <executable_name>`. So for our :file:`awesome_package` and its :file:`setpoint_publisher.py` it would be:

.. code-block:: sh

   ros2 run awesome_package setpoint_publisher.py

If you try to do so right now, you will likely get an error message :code:`No executable found`.

We created a package, but we haven't built our workspace since we modified :code:`CMakeLists.txt` (remember, that we are supposed to rebuild our workspace each time we modify this file?).

.. code-block:: sh

   build_ros

.. note:: Every time we create a new package, or create a new node in an existing package, we need to build our workspace with :code:`build_ros` and apply the updated package paths with :code:`. ~/.bashrc`. 

Now, we should be ready to finally run our code

.. code-block:: sh

   ros2 run awesome_package setpoint_publisher.py

.. hint:: You can use :kbd:`Tab` to use the shell's ability to auto-complete your commands. If the auto-completion is unambigous, a single hit will suffice. If there is more than one auto-complete option, hit :kbd:`Tab` twice to show the different options. 

.. hint:: Just to remind you: you stop running programs in a terminal by the shortcut :kbd:`Ctrl` + :kbd:`C`.

In the node's source code you can see that the sent setpoints are :code:`sin` and :code:`cos` signals.

We have started the :code:`setpoint_publisher.py` node.
But since it just publishes ROS messages we can't see any output in the terminals.
We can use command line tools :code:`ros2 node` and :code:`ros2 topic` to get some insights on what is going on in the background hidden from our curious eyes.
With :code:`ros2 node info /name/of/our/node` we can get various information on our node. For example what publications and what subscriptions it has.
Or in other words: what are the topics the node wants to receive data on and what are the topics it ouputs data on.

To get a list of all nodes, we run

.. code-block:: sh

   ros2 node list

which in our case should yield::

   /my_first_node

To get more information on this node, we run

.. code-block:: sh

   ros2 node info /my_first_node

which in turn yields

.. asciinema:: /res/asciinema/ros2_node_info.cast
   :speed: 2
   :start-at: 1
   :idle-time-limit: 1
   :poster: npt:0:01

.. hint::
   Again, we can use :kbd:`Tab` to auto-complete the node name after we have started writing the first few characters.
   Start using this feature if you haven't already! 

The first two publishers are internally created by ROS2.
We do not care about them for now.
The other publishers are the ones we have created with the program that we have written.

To see what messages the node is actually publishing, we could use :code:`ros2 topic echo /the/topic/name/to/echo`.

.. asciinema:: /res/asciinema/ros2_topic_echo.cast
   :speed: 2
   :start-at: 1
   :idle-time-limit: 1
   :poster: npt:0:01

.. note:: We ad :code:`--once` at the end of the command to echo only a single message. If you omit this argument, :code:`ros2 topic echo` will continue to print messages until you stop it with :kbd:`Ctrl` + :kbd:`C`. 

These two commands are great to get at least some insights on what is going on during the execution of our node.
But most of us will find it rather cumbersome to evaluate the echoed data in realtime.
I mean, would you claim to be able to see that the echoed data is actually the output of a sine function?
So, some proper plotting tool might come in handy here.

We can use :code:`plotjuggler` to visualize the data.
General information to :code:`plotjuggler` can be found on the `GitHub Page <https://facontidavide.github.io/PlotJuggler/visualization_howto/index.html>`__ and some step-by-step instructions in the section :ref:`tutorials/real_time_plotting:Real-Time Plotting`.
