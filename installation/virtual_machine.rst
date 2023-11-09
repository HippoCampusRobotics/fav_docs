Virtual Machine
###############

.. attention:: Skip this section if you do **not** want to use a virtual machine but you have installed Ubuntu directly on your computer (**recommended**). Continue with the :ref:`installation of ROS<installation/install_ros:Install ROS>`.

For instructions on how to install Ubuntu in a Virtual Machine please use your preferred search engine to find one of the many tutorials out there.

For performance reasons we recommend to run Ubuntu alongside Windows as Dual Boot as described in the :ref:`installation/install_ubuntu:Install Ubuntu` section. 

Troubleshooting
===============

If you can not launch the :code:`gazebo` command after you have installed :code:`ros-noetic-desktop-full` in the section :ref:`installation/install_ros:Install ROS` please execute the command

.. code-block:: console

   $ echo "export SVGA_VGPU10=0" >> ~/.profile

and restart your Virtual Machine.

