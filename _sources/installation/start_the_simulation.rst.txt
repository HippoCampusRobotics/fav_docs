Start The Simulation
####################

Now that we have everything prepared, i.e.:

* :ref:`Installation of ROS<installation/install_ros:Install ROS>` 

.. * :ref:`Installation of PX4 <installation/install_firmware:Install Firmware>`

we can eventually start to work with ROS and the Gazebo simulator.

At the time being, there are 5 launchfiles that come with the :file:`fav_sim` package.

.. code-block:: sh
   :emphasize-lines: 7, 11-15

   /home/YOUR_USER_NAME/fav/catkin_ws/src/fav
   ├── fav_gazebo_plugins
   │   ├── include
   │   └── src
   ├── fav_msgs
   │   └── msg
   └── fav_sim
      ├── config
      ├── include
      ├── launch
      │   ├── gazebo_apriltag_tank_world.launch
      │   ├── gazebo_empty_world.launch
      │   ├── keyboard_control.launch
      │   ├── simulation.launch
      │   └── spawn_vehicle.launch
      ├── models
      ├── nodes
      ├── res
      ├── src
      └── worlds



gazebo_empty_world.launch
   Launches gazebo with the basic world file in :file:`fav_sim/worlds/base.world`.

gazebo_apriltag_tank_world.launch
   Equivalent to :file:`gazebo_empty_world.launch` but also includes a tank with an AprilTag pattern on the ground.

spawn_vehicle.launch
   Spawns the BlueROV model. The model files are in :file:`fav_sim/models/bluerov`. This launch setup requires :code:`gazebo` to be already running. Otherwise no vehicle can be spawned.

simulation.launch
   Includes :file:`gazebo_apriltag_tank_world.launch` and :file:`spawn_vehicle.launch`. So instead of starting the others separately, it is possible to just launch this launch file.

keyboard_control.launch
   Starts a node to control the BlueROV via keyboard input. This launch setup requires :code:`gazebo` to be running and an already spawned BlueROV vehicle. Otherwise no vehicle can be controlled.

To test that everything is set up correctly, open two terminals (for example by using the integrated terminal in VS Code and hitting the **split terminal** button.

In the first terminal, enter:

.. code-block:: sh

   roslaunch fav_sim simulation.launch

This should launch the simulation environment (gazebo) and spawn the BlueROV robot model.

.. note:: In some rare cases, the Gazebo window will stay black. If this is the case, quit by hitting :kbd:`Ctrl` + :kbd:`C` in the terminal in which you started Gazebo and try again.

Lastly, run in another terminal:

.. code-block:: sh

   roslaunch fav_sim keyboard_control.launch

This creates a small window for keyboard control.

.. image:: /res/images/keyboard_control_qt.png


Make sure you have the keyboard control window in the foreground so the keyboard inputs get captured.

You can use the sliders to scale the thruster output for the different actuation channels to your liking.

Useful keys are :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D`, :kbd:`Left`, :kbd:`Right`, :kbd:`Up`, :kbd:`Down`. Use them and find out what they are doing |partying_face|.

Stop everything by hitting :kbd:`Ctrl` + :kbd:`C` in all terminals in which you have started launch setups.

.. note:: The keyboard control window reads your keyboard input. If the window is not the active one, the vehicle will not react to your input!


.. hint:: For assignment 0, it is not required to read the further sections. But if you like to continue, feel free to do so.
