Start The Simulation
####################

Now that we have everything prepared, i.e.:

* :ref:`installation/install_ros:Install ROS`
* :ref:`installation/workspace_setup:Workspace Setup`

we can finally start to work with ROS and the Gazebo simulator.
The directory structure should look similar to the below displayed one

.. code-block:: console
   :emphasize-lines: 5-6

   .
   └── fav
       ├── fav
       ├── launch
       │   ├── simulation.launch.py
       │   └── simulation_with_keyboard_control.launch.py
       └── keyboard_control

simulation.launch.py
   You probably recognize this one.
   We started this launch file at the end of our workspace setup to verify that everything was working correctly.

simulation_with_keyboard_control.launch.py
   Includes :file:`simulation.launch.py` as a child launch file and additionally starts a keyboard control node to remotely control the BlueROV.

Did we say we can remotely control the BlueROV in the simulation? Let's try it!

.. code-block:: console

   $ ros2 launch fav simulation_with_keyboard_control.launch.py vehicle_name:=bluerov00

Two windows should be created. One is the familiar simulation environment.
The second window is for the keyboard control.

.. note::

   If you are experiencing any difficulties, multiple warnings and errors appear within the terminal and gazebo dies after a few seconds, you may have a problem with your wayland protocol. In this case, you need to run the following command **each time** before you start the simulation with gazebo:

   .. code-block:: console

      $ export QT_QPA_PLATFORM=xcb

.. image:: /res/images/keyboard_control_qt.png

Make sure you have the keyboard control window in the foreground.
Otherwise, the keyboard inputs do not get captured.

You can use the sliders to scale the thruster output for the different actuation channels to your liking.

Useful keys are :kbd:`W`, :kbd:`A`, :kbd:`S`, :kbd:`D`, :kbd:`J`, :kbd:`L`, :kbd:`I`, :kbd:`K`.
Use them and find out what they are doing |partying_face|.

Stop everything by hitting :kbd:`Ctrl` + :kbd:`C` in the terminal in which you launched everything.
Alternatively, just close one of the windows.
This should result in everything shutting down as well.

.. note::

   The keyboard control window reads your keyboard input.
   If the window is not the active one, the vehicle will not react to your input!

.. hint::

   For assignment 0, it is not required to read the further sections.
   But if you like to continue, feel free to do so.
