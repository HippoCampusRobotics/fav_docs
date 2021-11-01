Assignment 1
############

.. attention:: 

   Since we are still working on the code as this class is progressing, please make sure to regularly update your local repositories.
   You can do this by navigating into the repository, using for example:

   .. code-block:: sh

      cd ~/fav/catkin_ws/src/bluerov_sim 

   To pull our changes, execute here:

   .. code-block:: sh

      git pull

   You will have to do this seperately for all of our repositories.

So you've learned some basics of ROS, or maybe you already know your way around ROS. Now it's finally time to get some hands-on experience with a real robot. 

You will design a depth controller for the BlueROV2. We have prepared an empty ROS package that you can use. To clone it, use:

.. code-block:: sh

   git clone https://github.com/FormulasAndVehicles/depth_control_template.git ~/fav/catkin_ws/src/depth_controller

This package already includes an *example* node for publishing depth setpoints.

To actuate the BlueROV's thrusters, you will need a mixer module. You can use the mixer module from :file:`bluerov_sim` in :file:`nodes/mixer.py`. Head over to :ref:`the_robot:The Robot` for a more detailed description of the BlueROV2 and its thruster configuration.

.. note::
   Remember to rebuild your catkin workspace after downloading or creating new packages. 
   You don't have to rebuild if you only change Python code.
   However, if you create a *new* Python file, you have to add it to :file:`CMakeLists.txt` and rebuild the workspace.

The Benefits of Simulation
**************************

In the real world it might be hard to tell if the depth of your vehicle calculated from pressure measurements is close to the real depth of the vehicle. In simulation we can compare the results with the ground truth information we get from the simulator itself. The topic name in our case is :file:`/bluerov/ground_truth/state` and has the type :file:`nav_msgs/Odometry`.