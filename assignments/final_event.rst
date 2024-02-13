Final Event
###########

Modify the Setup
================

For the demonstration of the final project in our lab, we need you to remove both the ``scenario_node`` and the ``robot_marker_publisher`` from your launch setup.
We will run these nodes on another machine.

.. attention::

   Not removing these nodes from your launch setup will cause node/topic collisions.

Remove this code block from ``final_project/launch/final_project.launch.py``

.. code-block:: python
   :linenos:
   :lineno-start: 74
   :caption: final_project/launch/final_project.launch.py

   Node(
       executable='scenario_node',
       package='fav',
       parameters=[
           {
               'scenario': LaunchConfiguration('scenario'),
               'use_sim_time': LaunchConfiguration('use_sim_time'),
           },
       ],
   ),
   Node(
       executable='robot_marker_publisher',
       package='fav',
       parameters=[
           {
               'use_sim_time': LaunchConfiguration('use_sim_time'),
           },
       ],
   ),



Since we will show the ``rviz`` visualization as well, you might consider removing the ``rviz`` node as well for performance reasons.
But that is up to your personal preferences.

.. code-block:: python
   :linenos:
   :lineno-start: 95
   :caption: final_project/launch/final_project.launch.py

   rviz_file = str(
      get_package_share_path('final_project') / 'config/rviz.rviz')

   action = Node(
      executable='rviz2',
      package='rviz2',
      name='rviz2',
      arguments=['-d', rviz_file, '--ros-args', '--log-level', 'error'],
   )
   launch_description.add_action(action)

Run the Code
============

Also keep in mind that we need to adapt the launch command to run the setup slightly, because we do not need sim time for the lab.
We do not need to specify a scenario either.

.. code-block:: console

   $ ros2 launch final_project final_project.launch.py vehicle_name:=bluerov01 use_sim_time:=false
