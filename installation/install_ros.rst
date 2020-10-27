Install ROS
##############

.. sectionauthor:: Lennart Alff <thies.lennart.alff@tuhh.de>

The `ROS Wiki <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ provides a complete installation guide. The following instructions are a summary of their article.

Preparation
===========

#. Add sources

   .. code-block:: sh

      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#. Add key

   .. code-block:: sh

      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


#. Update index

   .. code-block:: sh

      sudo apt update


Installation
============

#. Install ROS

   .. code-block:: sh

      sudo apt install ros-melodic-desktop-full

#. Install build dependencies

   .. code-block:: sh

      sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools

Set Up Catkin Workspace
=======================

#. Source your ROS installation

   .. code-block:: sh

      source /opt/ros/melodic/setup.bash

#. Create workspace

   .. code-block:: sh

      mkdir -p ~/fav/catkin_ws/src && cd ~/fav/catkin_ws

#. Initialize workspace

   .. code-block:: sh

      catkin init

#. Build empty workspace

   .. code-block:: sh

      catkin build

To automatically source your catkin workspace execute the following command:

.. code-block:: sh

   echo '$HOME/fav/catkin_ws/devel/setup.bash' >> ~/.bashrc

To apply this changes execute:

.. code-block:: sh

   source ~/.bashrc

For new terminal session your catkin workspace is sourced automatically since we added the instruction to do so to the :file:`.bashrc` file.