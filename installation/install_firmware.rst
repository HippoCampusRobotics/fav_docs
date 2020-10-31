Install Firmware
################

.. sectionauthor:: Lennart Alff <thies.lennart.alff@tuhh.de>


.. note:: The BlueROV used in this class runs the PX4 firmware. For the class we have created a fork of the firmware, please follow the instructions to install it. Most likely you will not get in touch with any of the Firmware source code, but still you need to clone and install the corresponding repositories to complete this setup guide.

Set Up The Workspace
====================

By now your directory structure should look like this if you followed the previous ROS installation section::

   ~/fav
   └── catkin_ws
       ├── build
       ├── devel
       ├── logs
       └── src

#. Clone the Firmware

   .. code-block:: bash

      cd ~/fav && git clone --recursive https://github.com/HippoCampusRobotics/fav_PX4-Autopilot.git

#. Clone MAVLink

   .. code-block:: bash

      git clone https://github.com/HippoCampusRobotics/fav_mavlink.git ~/fav/catkin_ws/src/mavlink

#. Clone mavros

   .. code-block:: bash

      git clone https://github.com/HippoCampusRobotics/fav_mavros.git ~/fav/catkin_ws/src/mavros

#. Install GeographicLib dataset

   .. code-block:: bash

      sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh



Now your directory structure should look similiar to::

   ~/fav
   ├── catkin_ws
   │   ├── build
   │   ├── devel
   │   ├── logs
   │   └── src
   │       ├── mavlink
   │       └── mavros
   └── fav_PX4-Autopilot

Build The Catkin Workspace
==========================

#. Switch into your :file:`catkin_ws`

   .. code-block:: bash

      cd ~/fav/catkin_ws

#. Build the code

   .. code-block:: bash

      catkin build

Build The PX4 Firmware
======================

#. Switch into your :file:`fav_PX4-Autopilot` directory

   .. code-block:: bash

      cd ~/fav/fav_PX4-Autopilot

#. Set up dependencies

   .. code-block:: bash

      bash ./Tools/setup/ubuntu.sh --no-nuttx

#. Build the code

   .. code-block:: bash

      DONT_RUN=1 make px4_sitl gazebo_uuv_bluerov2_heavy

Configure Environment Variables
===============================

#. Switch into your :file:`fav` directory

   .. code-block:: bash

      cd ~/fav

#. Edit your :file:`~/.bashrc` by executing the following commands

   .. code-block:: bash

      echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc

      echo "source $(pwd)/fav_PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/fav_PX4-Autopilot $(pwd)/fav_PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc

      echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/fav_PX4-Autopilot" >> ~/.bashrc

      echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/fav_PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

#. Apply the changes of :file:`~/.bashrc`

   .. code-block:: bash

      source ~/.bashrc

Test Your Setup
===============

At this point you can check that your setup is functional by launching the simulation:

.. code-block:: bash

   roslaunch px4 mavros_posix_sitl.launch vehicle:=uuv_bluerov2_heavy world:=~/fav/fav_PX4-Autopilot/Tools/sitl_gazebo/worlds/uuv_hippocampus.world

After a few seconds something similiar to the following screenshot should appear:

.. image:: /res/images/px4_test_screenshot.png

You can quit the program by hitting :kbd:`Ctrl` + :kbd:`C` in the terminal you have started it. It will take a few seconds to stop all the running processes.

.. note:: Closing the Gazebo GUI will **only** close the Gazebo GUI. All the other started processes will continue running. So please use the above mentioned shortcut to kill them.




