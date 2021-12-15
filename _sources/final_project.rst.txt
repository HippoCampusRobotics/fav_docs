Final Project
#############

.. .. attention:: 

..    Please make sure to update your local repository.

..    .. code-block:: sh

..       cd ~/fav/catkin_ws/src/bluerov_sim 

..    To pull our changes, execute:

..    .. code-block:: sh

..       git pull origin --ff-only

This section presents some additional features to our simulation environment. It should give you an idea on how to add models to the Gazebo world for your final project.

Additionally, note that you can use the position data of our localization algorithm in experiment. This will allow you to focus on other tasks.


New Gazebo World
================

There is a new world-file in :file:`bluerov_sim` called :file:`tank_with_tags.world`. It is a predefined world that contains a model of the tank with AprilTags on the floor, similiar to the real tank at the institute.

To load this world, when we launch Gazebo, we can include the :file:`gazebo_tags.launch` instead of the :file:`gazebo_base.launch` launch file.

A complete launch setup is provided in :file:`example_apriltag_world.launch`.

.. code-block:: sh

   roslaunch bluerov_sim example_apriltag_world.launch

It launches Gazebo with the AprilTags in the tank and spawns the BlueROV.

BlueROV2 Model including Cameras
================================

There is a new model of the BlueROV2 that has two cameras. To spawn this variant of the vehicle, pass the :code:`camera` argument to the :file:`spawn_vehicle.launch` launch file and set it to true.

.. image:: /res/images/gazebo_camera.png


:file:`example_apriltag_world.launch` provides the possibility to hand over this argument via command line. It is set to :code:`false` by default, so you only have to specify its value, if you want to set it to :code:`true`.

.. code-block:: sh

   roslaunch bluerov_sim example_apriltag_world.launch camera:=true

Line 9 creates a :code:`camera` argument (that can be specified via commandline and defaults to :code:`false`) that is passed to :file:`spawn_vehicle.launch` in line 15.

.. code-block:: xml
   :linenos:
   :emphasize-lines: 9,15

   <launch>
      <!-- Vehicle spawn position so that tags are seen -->
      <arg name="x" default="0.7" />
      <arg name="y" default="2.0" />
      <arg name="z" default="-0.7" />
      <arg name="P" default="0.0" />
      <arg name="R" default="0.0" />
      <arg name="Y" default="1.57" />
      <arg name="camera" default="false" />

      <include file="$(find bluerov_sim)/launch/gazebo_tags.launch" />

      <!-- spawn BlueROV model -->
      <include file="$(find bluerov_sim)/launch/spawn_vehicle.launch">
         <arg name="camera" value="$(arg camera)" />
         <arg name="x" value="$(arg x)" />
         <arg name="y" value="$(arg y)" />
         <arg name="z" value="$(arg z)" />
         <arg name="R" default="$(arg R)" />
         <arg name="P" default="$(arg P)" />
         <arg name="Y" default="$(arg Y)" />
      </include>
   </launch>

.. hint:: Feel free to modify the camera parameters at the end of the file :file:`bluerov_sim/models/uuv_bluerov2_heavy_cam/uuv_bluerov2_heavy_cam.sdf`. 

New AprilTag Models
===================

In :file:`bluerov_sim/models` there are many new AprilTag models. 128 models in total now to be precise. IDs 0 to 63 are used for the tags on the floor.

Modify Gazebo Worlds
====================

In general you have two options to get your models into a Gazebo world. Either you use predefined world files like the already mentioned :file:`tank_with_tags.world` and specify the models you want to include (lines 22-24).

.. code-block:: xml
   :linenos:
   :emphasize-lines: 22-24

   <?xml version="1.0"?>
   <sdf version="1.6">
      <world name="base">
         <include>
               <uri>model://sun</uri>
         </include>
         <!-- second sun to illuminate Tags -->
         <light type="directional" name="sun2">
               <cast_shadows>true</cast_shadows>
               <pose>0 0 10 0 0 0</pose>
               <diffuse>0.8 0.8 0.8 1</diffuse>
               <specular>0.2 0.2 0.2 1</specular>
               <attenuation>
               <range>1000</range>
               <constant>0.9</constant>
               <linear>0.01</linear>
               <quadratic>0.001</quadratic>
               </attenuation>
               <direction>0.0 1.0 0.0</direction>
         </light>

         <include>
               <uri>model://apriltag_tank</uri>
         </include>
      </world>
   </sdf>

Or you spawn models during runtime (like it is done for the BlueROV model for example). The :file:`gazebo_ros` package provides functionality for this

Simply start the node in your launch file and pass it the path to the model's sdf-file you want to spawn. You can also define the initial pose.

To spawn the apriltag with ID 127 you can add the following lines to your launch file.

.. code-block:: xml

   <node name="choose_arbitrary_name" pkg="gazebo_ros" type="spawn_model"
       args="-sdf -file $(find bluerov_sim)/models/tag36_11_00127/model.sdf 
             -model tag_127
             -x 0.1 -y 0.3 -z -0.5
             -R 0.9 -P 0.2 -Y 2.4" />
            
Or if you have Gazebo already running, you can enter

.. code-block:: sh

   SDF_MODEL=$(rospack find bluerov_sim)/models/tag36_11_00127/model.sdf
   rosrun gazebo_ros spawn_model -sdf -file $SDF_MODEL -model tag_127-x0.1 -y0.3 -z-0.5 -R0.9 -P0.2 -Y2.4

to spawn the apriltag.

.. image:: /res/images/gazebo_spawned_tag.png


