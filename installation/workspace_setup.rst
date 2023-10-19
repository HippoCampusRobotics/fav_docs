Workspace Setup
###############

Create the Structure
====================

.. code-block:: sh

   mkdir -p ~/fav/ros2/src
   mkdir -p ~/fav/ros2_underlay/src

In ROS, we put the code we write (or use) in so-called "workspaces". 
We can have multiple workspaces and can either use them in combination by "overlaying" the workspaces. Or we can have them as completely independent setups and use only a certain selection of them at a time.

For this class, we use the two workspaces :file:`ros2` and :file:`ros2_underlay`. As the name suggests, we use :file:`ros2` as an overlay to :file:`ros2_underlay`. Of course there is a reasoning behind this.

ros2_underlay
   We use this workspace for code/packages we want or have to build from source. However, these packages we do not modify regularly or work with at all. Outsourcing these packages into a separate workspace speeds up the compilation time of our actual development workspace.

ros2
   This is our development workspace. We put all the packages/code developed during this class here.

1. "ros2_underlay" workspace
============================

We will first setup the **ros2_underlay** workspace. The setup includes three steps:

- *downloading* the code of the packages we want from e.g. github
- *building* the code
- *sourcing* the workspace, so that the built packages can be found by your machine and executed


Populate the "ros2_underlay" workspace
**************************************

Let's download the relevant packages

.. code-block:: sh

   cd ~/fav/ros2_underlay/src \
   && git clone --recursive https://github.com/HippoCampusRobotics/hippo_core.git \
   && git clone https://github.com/HippoCampusRobotics/hippo_simulation.git

.. code-block:: sh

   cd ~/fav/ros2_underlay/src \
   && git clone https://github.com/PX4/px4_msgs.git && \
   cd px4_msgs && \
   git checkout 8a7f3da

.. todo::
   Do we need apriltag_ros here?
   If so, we probably want to have our own fork.


Build the "ros2_underlay" workspace
**************************************

Create an *alias* for the build command for convenience

.. code-block:: sh

   echo "alias build_underlay=\"env -i HOME=\$HOME USER=\$USER TERM=xterm-256color zsh -l -c 'source /opt/ros/iron/setup.zsh && cd \$HOME/fav/ros2_underlay && colcon build'\"" >> ~/.zshrc
   source ~/.zshrc

Make sure dependencies are installed

.. code-block:: sh

   cd ~/fav/ros2_underlay \
   && source /opt/ros/iron/setup.zsh \
   && rosdep install --from-paths src -y --ignore-src

Build the workspace (this may take some time!):

.. code-block:: sh

   build_underlay

Note that by executing the defined alias, you do not have to be inside the respective workspace directory to build. Very convenient!

Source the "ros2_underlay" workspace
**************************************

.. code-block:: sh

   echo 'source "$HOME/fav/ros2_underlay/install/setup.zsh"' >> ~/.zshrc && \
   source ~/.zshrc

2. "ros2" workspace
====================

Now on to our development workspace. You will later on fill this workspace with your own packages. Exciting!

Populate "ros2" workspace
**************************************

.. code-block:: sh

   cd ~/fav/ros2/src \
   && git clone https://github.com/FormulasAndVehicles/fav.git

Build "ros2" workspace
**************************************

Create an alias for the build command for convenience

.. code-block:: sh

   echo "alias build_ros=\"env -i HOME=\$HOME USER=\$USER TERM=xterm-256color zsh -l -c 'source \$HOME/fav/ros2_underlay/install/setup.zsh && cd \$HOME/fav/ros2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'\"" >> ~/.zshrc
   source ~/.zshrc

Make sure dependencies are installed

.. code-block:: sh

   cd ~/fav/ros2 \
   && rosdep install --from-paths src -y --ignore-src

Build the workspace (this may take some time!):

.. code-block:: sh

   build_ros



Source "ros2" workspace
**************************************

.. code-block:: sh

   echo 'source "$HOME/fav/ros2/install/local_setup.zsh"' >> ~/.zshrc \
   && . ~/.zshrc



Check :file:`.zshrc` file
==========================

.. note:: 
   
   The mysterious :file:`.zshrc` file is used to set environment variables.
   Everyone sometimes makes typos or errors while adding stuff to this :file:`.zshrc` file. You might open a new terminal and get an error as follows on the top:

   .. code-block:: sh

      bash: /...[file path+name]: no such file or directory

   
      
   In case you echo (aka write) a wrong line into the :file:`.zshrc` file, you can delete this line by opening the file in your favorite text editor. For example, using :code:`gedit` as text editor: 

   .. code-block:: sh
      
      gedit ~/.zshrc

   The file should open in a new window. The command :code:`echo` adds lines to the end of the file. Therefore, scroll to the bottom and find what you've added. Delete the lines that cause errors and save the changes.


Open the :file:`.zshrc` file, for example using :code:`gedit` as text editor:

.. code-block:: sh

   gedit ~/.zshrc

Your :file:`~/.zshrc` should look like this this for the last lines:

.. code-block:: 

   ...

   source /opt/ros/iron/setup.zsh
   alias build_underlay="env -i HOME=$HOME USER=$USER TERM=xterm-256color zsh -l -c 'source /opt/ros/iron/setup.zsh && cd $HOME/fav/ros2_underlay && colcon build'"
   source "$HOME/fav/ros2_underlay/install/setup.zsh"
   alias build_ros="env -i HOME=$HOME USER=$USER TERM=xterm-256color zsh -l -c 'source $HOME/fav/ros2_underlay/install/setup.zsh && cd $HOME/ros2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'"
   source "$HOME/fav/ros2/install/local_setup.zsh"

Final Check
===========

Are we sure that everything is prepared correctly? Let's check it!

.. code-block:: sh

   ros2 launch fav simulation.launch.py vehicle_name:=bluerov00

If a window similar to the following opens, we are on the right track

.. image:: /res/images/gazebo_test.png

If things do not work out as you hoped, please read the terminal output carefully and check for errors. If you cannot figure out the problem yourself, send a copy of the complete output to your favourite research associate. Preferably via Slack. And use the code-block function there (:kbd:`Ctrl` + :kbd:`Alt` + :kbd:`Shift` + :kbd:`C`). This really helps us to help you!
