Start The Simulation
####################

Now that we have everything prepared, i.e.:

* :ref:`Installation of ROS<installation/install_ros:Install ROS>` 

.. * :ref:`Installation of PX4 <installation/install_firmware:Install Firmware>`

we can eventually start to work with ROS and the Gazebo simulator.

At the time being there are 4 launchfiles that come with the :file:`fav_sim` package.

gazebo_empty_world.launch
   Launches gazebo with the basic world file in :file:`fav_sim/worlds/base.world`.

spawn_vehicle.launch
   Spawns the BlueROV model. The model files are in :file:`fav_sim/models/bluerov`

keyboard_control.launch
   Includes the :file:`spawn_vehicle.launch` file and starts a node to control the BlueROV via keyboard input.

To test that everything is set up correctly open two terminals (for example by using the integrated terminal in VS Code and hitting the **split terminal** button.

In the first terminal, enter:

.. code-block:: sh

   roslaunch fav_sim gazebo_apriltag_tank_world.launch

This should launch Gazebo. Wait until an empty world is visible.

.. note:: In some rare cases the Gazebo window will stay black. If this is the case, quit by hitting :kbd:`Ctrl` + :kbd:`C` in the terminal in which you started Gazebo and try again.

In the second enter:

.. code-block::

   roslaunch fav_sim keyboard_control.launch

This will spawn the BlueROV and creates a small window for keyboard control:

.. image:: /res/images/keyboard_control_qt.png


Make sure you have the keyboard control window in the foreground so the keyboard inputs get captured.

You can use the sliders to scale the thruster output for the different actuation channels to your liking.

Useful keys are :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D`, :kbd:`Left`, :kbd:`Right`, :kbd:`Up`, :kbd:`Down`. Use them and find out out what they are doing :-)

Stop everything by hitting :kbd:`Ctrl` + :kbd:`C` in both terminals.

.. note:: The keyboard control window reads your keyboard input. If the window is not the active one, the vehicle will not react to your input!


.. hint:: For assignment 0 it is not required to read the further sections. But if you like to continue, feel free to do so.

If you are done, you can stop everything by hitting :kbd:`Ctrl` + :kbd:`C` in both terminals.