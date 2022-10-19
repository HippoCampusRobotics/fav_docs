Install ROS
###########

The `ROS Wiki <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ provides a complete installation guide. The following instructions are a summary of their article.

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

      sudo apt install curl # if you haven't already installed curl
      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


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

      sudo apt install ros-noetic-desktop-full

#. Install build dependencies

   .. code-block:: sh

      sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

#. Install catkin-tools

   .. code-block:: sh

      sudo apt install python3-catkin-tools python3-osrf-pycommon

#. Set up :code:`rosdep`

   .. code-block:: sh

      sudo rosdep init

   .. code-block:: sh

      rosdep update

Set Up Catkin Workspace
=======================

#. Source your ROS installation

   .. code-block:: sh

      source /opt/ros/noetic/setup.bash

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

   echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
   echo 'source $HOME/fav/catkin_ws/devel/setup.bash' >> ~/.bashrc

To apply these changes, execute:

.. code-block:: sh

   source ~/.bashrc

For new terminal session your catkin workspace is sourced automatically since we added the instruction to do so to the :file:`.bashrc` file.

.. attention::

   If you are using :code:`zsh` instead of :code:`bash`, you need to adjust the above commands. You will also have pay attention and adjust some commands in the following parts of this setup guide.

Get The First Catkin Packages
=============================

#. Make sure you are inside your catkin workspace:

   .. code-block:: bash

      cd ~/fav/catkin_ws

#. Clone the :file:`fav` repository

   .. code-block:: bash

      git clone https://github.com/FormulasAndVehicles/fav.git ~/fav/catkin_ws/src/fav

   At this stage make sure your directory structure looks similar to this:

   .. code-block:: sh
      
      /home/YOUR_USER_NAME/fav/catkin_ws
      ├── build
      ├── devel
      ├── logs
      └── src
          └── fav
              ├── fav_gazebo_plugins
              ├── fav_msgs
              └── fav_sim



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

Test the Setup
==============

.. hint:: If launching gazebo from ROS, ROS is not able to kill the gazebo node without escalating to **SIGTERM**. This happens after a 15s timeout. If you do not have the time to wait for so long, you can modifiy :file:`/opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py` and change :code:`DEFAULT_TIMEOUT_SIGINT` to some value you are willing to wait.

If you are willing to wait, let's say, 3.0 seconds, execute the following command:

.. code-block:: sh

   sudo sed -i 's/DEFAULT_TIMEOUT_SIGINT\s*=\s*15\.0.*/DEFAULT_TIMEOUT_SIGINT = 3\.0/' /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py 


Execute the following command

.. code-block:: sh

   roslaunch fav_sim gazebo_apriltag_tank_world.launch


and :code:`gazebo` should launch after a few seconds and look this:

.. image:: /res/images/gazebo_tags.png

If the :code:`gazebo` window stays black, enter :kbd:`Ctrl` + :kbd:`C` in the terminal in which you executed the command above and try it again.
