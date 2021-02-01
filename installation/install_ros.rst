Install ROS
###########

.. sectionauthor:: Lennart Alff <thies.lennart.alff@tuhh.de>

The `ROS Wiki <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ provides a complete installation guide. The following instructions are a summary of their article.

.. attention:: 

   Since Ubuntu 18.04, Canonical decided it was a good idea to have unattended updates running in the background. This might lead to error messages similiar to:

   .. code-block:: sh

      E: Could not get lock /var/lib/dpkg/lock-frontend - open (11: Resource temporarily unavailable)
      E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
   
   If you encounter them after running anything related to :code:`apt`: no worries. Just wait until the backgroud updates are finished.

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


#. Make sure your package are up to date!(**!!!**)

   .. code-block:: sh

      sudo apt upgrade -y


Installation
============

#. Install ROS

   .. code-block:: sh

      sudo apt install ros-melodic-desktop-full

#. Install build dependencies

   .. code-block:: sh

      sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools

#. Set up :code:`rosdep`

   .. code-block:: sh

      sudo rosdep init

   .. code-block:: sh

      rosdep update

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

To automatically source your catkin workspace, execute the following command:

.. code-block:: sh

   echo 'source $HOME/fav/catkin_ws/devel/setup.bash' >> ~/.bashrc

To apply these changes, execute:

.. code-block:: sh

   source ~/.bashrc

For new terminal session your catkin workspace is sourced automatically since we added the instruction to do so to the :file:`.bashrc` file.

Get The First Catkin Package
============================

#. Make sure you are inside your catkin workspace:

   .. code-block:: bash

      cd ~/fav/catkin_ws

#. Clone :file:`bluerov_sim`

   .. code-block:: bash

      git clone https://github.com/FormulasAndVehicles/bluerov_sim.git src/bluerov_sim

#. Let ROS resolve the package's dependencies

   .. code-block:: bash

      rosdep install --from-paths src --ignore-src -r -y

#. Rebuild your workspace

   .. code-block:: bash

      catkin build

#. Apply changes of your environment variables by either

   * starting a new terminal session (remember that this means :file:`~/.bashrc` gets sourced automatically as mentioned before).

   * sourcing :file:`~/.bashrc` manually by executing

      .. code-block:: bash

         source ~/.bashrc
