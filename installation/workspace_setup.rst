Workspace Setup
###############

Create the Structure
====================

.. code-block:: sh

   mkdir -p ~/fav/ros2/src
   mkdir -p ~/fav/ros2_underlay/src

In ROS we put our code we write (or use) in workspaces. We can have multiple of them and can either use them in combination by overlaying the workspaces or we can have them as completely independent setups and use only a certain selection of them at a time.

For the class we use the two workspaces :file:`ros2` and :file:`ros2_underlay`. As the name suggests, we use :file:`ros2` as an overlay to :file:`ros2_underlay`. Of course there is a reasoning behind this.

ros2_underlay
   We use this workspace for code/packages we want/have to build from source but do not modify them regularly or work with them at all. Outsourcing these packages into a separate workspace speeds up the compilation time of our actual development workspace. 

ros2
   This is our development workspace. We put all the packages/code developed during this class here.

Populate ros2_underlay
======================

.. todo::
   Those packages are actually not rquired.
      * px4_msgs can be installed via package.
      * The pre-compiled package of plotjuggler seems to work wihout problems since iron.

px4_msgs
********

.. code-block:: sh

   cd ~/fav/ros2_underlay/src && \
   git clone https://github.com/PX4/px4_msgs.git && \
   cd px4_msgs && \
   git checkout 8a7f3da

apriltag_ros
************

.. code-block:: sh

   cd ~/fav/ros2_underlay/src && \
   git clone --depth 1 --branch ros2-port https://github.com/wep21/apriltag_ros.git

plotjuggler
***********

.. code-block:: sh

   cd ~/fav/ros2_underlay/src && \
   git clone https://github.com/PlotJuggler/plotjuggler_msgs.git && \
   git clone --depth 1 --branch 1.7.3 https://github.com/PlotJuggler/plotjuggler-ros-plugins.git && \
   git clone --depth 1 --branch 3.7.1 https://github.com/facontidavide/PlotJuggler.git

Build ros2_underlay
===================
Create an alias for the build command for convenience

.. code-block:: sh

   echo "alias build_underlay=\"env -i HOME=\$HOME USER=\$USER TERM=xterm-256color bash -l -c 'source /opt/ros/iron/setup.bash && cd \$HOME/fav/ros2_underlay && colcon build'\"" >> ~/.bashrc
   source ~/.bashrc

Make sure dependencies are installed

.. code-block:: sh

   cd ~/fav/ros2_underlay \
   && source /opt/ros/iron/setup.bash \
   && rosdep install --from-paths src -y --ignore-src

Build the workspace (this may take some time!):

.. code-block:: sh

   build_underlay

Note that you do not have to be inside the respective workspace directory to build by executing the defined alias. Very convenient!

Source ros2_underlay
====================

.. code-block:: sh

   echo 'source $HOME/fav/ros2_underlay/install/setup.bash' >> ~/.bashrc && \
   source ~/.bashrc

Populate ros2
=============

.. code-block:: sh

   cd ~/fav/ros2/src \
   && git clone --recursive https://github.com/HippoCampusRobotics/hippo_core.git \
   && git clone https://github.com/HippoCampusRobotics/hippo_simulation.git \
   && git clone https://github.com/FormulasAndVehicles/fav.git

Build ros2
==========

Create an alias for the build command for convenience

.. code-block:: sh

   echo "alias build_ros=\"env -i HOME=\$HOME USER=\$USER TERM=xterm-256color bash -l -c 'source \$HOME/fav/ros2_underlay/install/setup.bash && cd \$HOME/fav/ros2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'\"" >> ~/.bashrc
   source ~/.bashrc

Make sure dependencies are installed

.. code-block:: sh

   cd ~/fav/ros2 \
   && rosdep install --from-paths src -y --ignore-src

Build the workspace (this may take some time!):

.. code-block:: sh

   build_ros

.. code-block:: sh

   echo 'source $HOME/ros2/install/local_setup.bash' >> ~/.bashrc \
   && . ~/.bashrc

Check :file:`.bashrc`
=====================

Your :file:`~/.bashrc` should look like this this for the last lines:

.. code-block:: 

   ...


   alias build_ros="env -i HOME=$HOME USER=$USER TERM=xterm-256color bash -l -c 'source $HOME/fav/ros2_underlay/install/setup.bash && cd $HOME/ros2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'"
   alias build_underlay="env -i HOME=$HOME USER=$USER TERM=xterm-256color bash -l -c 'source /opt/ros/iron/setup.bash && cd $HOME/fav/ros2_underlay && colcon build'"

   source /opt/ros/iron/setup.bash
   source $HOME/fav/ros2_underlay/install/setup.bash
   source $HOME/fav/ros2/install/local_setup.bash

Final Check
===========

Are we sure, everything is prepared correctly? Let's check it!

.. code-block:: sh

   ros2 launch fav simulation.launch.py vehicle_name:=bluerov00

If a window similar to the following opens, we are on the right track

.. image:: /res/images/gazebo_test.png

If things do not work out as you hoped, please read the terminal output carefully and check for errors. If you cannot figure out the problem yourself, send a copy of the complete output to your favourite research associate. Preferably via Slack. And use the code-block function there (:kbd:`Ctrl` + :kbd:`Alt` + :kbd:`Shift` + :kbd:`C`). This really helps us to help you.
