Start The Simulation
####################

.. sectionauthor:: Lennart Alff <thies.lennart.alff@tuhh.de>

Now that we have everything prepared, i.e.:

* :ref:`Installation of ROS<installation/install_ros:Install ROS>` 

* :ref:`Installation of PX4 <installation/install_firmware:Install Firmware>`

we can eventually start to work with ROS and the Gazebo simulator.

At the time being there are 3 launchfiles that come with the :file:`bluerov_sim` package.

gazebo_base.launch
   Launches gazebo with the basic world file in :file:`bluerov_sim/worlds/base.world`

spawn_vehicle.launch
   Spawns the BlueROV model. The model files are in :file:`bluerov_sim/models/uuv_bluerov2_heavy`

keyboard_control.launch
   Includes both the above mentioned launchfiles and also starts a node to control the BlueROV.

To test that everything is set up correctly open two terminals (for example by using the integrated terminal in VS Code and hitting the **split terminal** button.

In the first terminal, enter:

.. code-block:: sh

   roslaunch bluerov_sim gazebo_base.launch

This should launch Gazebo. Wait until an empty world is visible.

.. note:: In some rare cases the Gazebo window will stay black. If this is the case, quit by hitting :kbd:`Ctrl` + :kbd:`C` in the terminal in which you started Gazebo and try again.

In the second enter:

.. code-block::

   roslaunch bluerov_sim spawn_vehicle.launch

You should see a model of the BlueROV appearing in your Gazebo window.

Stop everything by hitting :kbd:`Ctrl` + :kbd:`C` in both terminals.

The Fun Part
============

If you are reading this and did not skip essential parts of the setup instructions we are ready to have some fun.

Launch the third of the mentioned launchfiles:

.. code-block:: sh

   roslaunch bluerov_sim keyboard_control.launch

This will start Gazebo, spawn the BlueROV and creates a small window for keyboard control.

Make sure you have the keyboard control window active:

.. image:: /res/images/keyboard_control.png

The controls are:

W/S:
   Move forward/backwards.

A/D:
   Move left/right.

Up/Down:
   Move upwards/downwards.

Left/Right:
   Turn left/right.

With the number keys you can decrease/increase the speed of movements.

.. note:: The keyboard control window reads your keyboard input. If the window is not the active one, the vehicle will not react to your input!




