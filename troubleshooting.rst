Troubleshooting
###############

What You Should Try First
=========================

Make sure your workspace is built

.. code-block:: console

   $ build_ros

and make sure your workspace is sourced after you added new packages/nodes!

.. code-block:: console

   $ source ~/.zshrc

I cannot run my node!
=====================

Problem
*******

You get a python exceptions with the last line of

.. code-block:: console

   Traceback (most recent call last):
   ...

being the following error

.. code-block:: console

   OSError: [Errno 8] Exec format error: <path/to/your/node>

Solution
********

You have forgotten to add

.. code-block:: python

   #!/usr/bin/env python3

as first line of your node.
Better add it now.


Problem
*******

You get the output

.. code-block:: console

   No executable found

Solution
********

Double check that the node exists, gets installed (``CMakeLists.txt``, see :ref:`tutorials/ros_package:Run A Node`), and is executable.
If you have forgotten to make it executable (most likely reason of the before mentioned causes), run 

.. code-block:: console

   $ chmod +x <PATH_TO_YOUR_NODE>


I cannot start the simulation!
===============================

Problem
********

You are using Ubuntu in a Virtual Machine and Gazebo dies after a few seconds and you get the following Output:

.. code-block:: console

   [gz-4] terminate called after throwing an instance of ‘Ogre::RenderingAPIException’
   [gz-4]   what():  OGRE EXCEPTION(3:RenderingAPIException): Fragment Program 100000002PixelShader_ps failed to compile. See compile log above for details. in GLSLShader::compile at ./.obj-aarch64-linux-gnu/gz_ogre_next_vendor-prefix/src/gz_ogre_next_vendor/RenderSystems/GL3Plus/src/GLSL/OgreGLSLShader.cpp (line 361)

The VM is trying to render with hardware to which is does not have full access.

Solution
**********

Run the following command:

.. code-block:: console

   $ echo ‘export LIBGL_ALWAYS_SOFTWARE=1’ >> ~/.zshrc \
   && . ~/.zshrc


My Robot/Gazebo is doing crazy stuff!
=====================================

Problem
*******

The robot shoots of to e.g. ``-1032839 m``, Gazebo is doing weird stuff, things aren't behaving at all how you would expect them.

This can happen if old Gazebo processes are still running in the background.
For example, this may happen after closing the simulation window, after a crash, or after stopping a launch file with ``Crtl + C``.

Solution
********

Close all leftover Gazebo processes.

We recommend using ``htop`` as a task manager for this.
It is easier to use than the default ``top`` command, but it is usually not installed by default.

Install it with:

.. code-block:: console

   $ sudo apt update && apt install htop


Start the task manager with

.. code-block:: console

   $ htop

Search for Gazebo processes by pressing ``F3`` and typing

.. code-block:: console
   
   gz


It should look similar to this: 

.. image:: /res/images/screenshot_htop_kill_gz.png


Look for processes such as

.. code-block:: console

   gz
   gz sim
   gzserver
   gzclient
   gazebo

Select the process with the arrow keys and press ``F9`` to kill it.
First try ``SIGTERM`` and press ``Enter``.

If the process does not disappear, press ``F9`` again, (select ``SIGKILL`` if necessary), and press ``Enter``.
Repeat this until no old Gazebo processes are left.

You can leave ``htop`` by pressing ``q``. Then start the simulation again.


If this does not fix it
***********************

If restarting Gazebo cleanly does not solve the problem, then the issue is probably not a leftover process.

Some other possible causes are:

* a controller applies very large forces, velocities, or accelerations
* ROS2 "multiple host interference": you see someone else's control commands as well - see below.



My robot reacts although I am not controlling it!
=================================================

Problem
*******

The robot moves although you are not sending commands, or it reacts in a way that does not match your own node.

This can happen if someone else on the same network is running ROS2 with the same topic names.
By default, ROS 2 uses ``ROS_DOMAIN_ID=0``.
If several students use the same network and the same domain ID, their nodes may discover each other and communicate.

For example, another student may publish control commands to the same topic that your robot is listening to.

Solution
********

Use a unique ``ROS_DOMAIN_ID`` for your own simulation.

Choose a number between ``1`` and ``101``.
Do not use ``0`` in class unless instructed otherwise.

For example, to use domain ID ``42`` temporarily in the current terminal, run:

.. code-block:: console

   $ export ROS_DOMAIN_ID=100

You must do this before starting your ROS2 nodes or launch files.

To make this permanent for every new terminal, add it to your ``~/.zshrc``:

.. code-block:: console

   $ echo 'export ROS_DOMAIN_ID=42' >> ~/.zshrc
   $ source ~/.zshrc

Check that it is set correctly with:

.. code-block:: console

   $ echo $ROS_DOMAIN_ID

You should see:

.. code-block:: console

   100

Important
*********

All terminals that belong to your own simulation must use the same ``ROS_DOMAIN_ID``.

If you are working with multiple computers, for example a robot computer and a laptop, both computers must use the same ``ROS_DOMAIN_ID`` if they are supposed to communicate.
In the lab, we use ``ROS_DOMAIN_ID=0``.

After changing the domain ID, stop all running ROS2 nodes and Gazebo processes, open a new terminal, and start the simulation again.