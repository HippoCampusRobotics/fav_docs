Final Project
#############

.. attention:: 

   Please make sure to update your local repository.

   .. code-block:: sh

      cd ~/fav/catkin_ws/src/fav 

   To pull our changes, execute:

   .. code-block:: sh

      git pull origin --ff-only

This section presents some additional features of our simulation environment. It should give you an idea on how to add models to the Gazebo world for your final project.

Additionally, note that you can use the position data of our localization algorithm in experiment. This will allow you to focus on other tasks.


Default World
=============

Probably the default world you want to start off with is :file:`tank_with_tags.world`. You already know the corresponding launch command from assignment 1.

.. code-block:: sh

   roslaunch fav_sim gazebo_apriltag_tank_world.launch


BlueROV2 Model with Simulated Cameras
=====================================

The good old description files for the BlueROV have changed. Optional cameras have been added. To enable them, we have the two launch arguments :code:`use_front_camera` and :code:`use_vertical_camera` available. Their default value is :code:`false`. To enable them, spawn the vehicle as follows:

.. code-block:: sh

   roslaunch fav_sim spawn_vehicle.launch use_front_camera:=true use_vertical_camera:=true

See :file:`spawn_vehicle.launch` for reference:

.. code-block:: xml
   :linenos:
   :emphasize-lines: 3,4

   <launch>
      <arg name="vehicle_name" default="bluerov" />
      <arg name="use_front_camera" default="false" />
      <arg name="use_vertical_camera" default="false" />
      <!-- position -->
      <arg name="x" default="0.5" />
      <arg name="y" default="1.0" />
      <arg name="z" default="-0.5" />
      <!-- roll, pitch, yaw -->
      <arg name="R" default="0.0" />
      <arg name="P" default="0.0" />
      <arg name="Y" default="1.57" />
      <group ns="$(arg vehicle_name)">
         <param name="robot_description" command="$(find xacro)/xacro $(find fav_sim)/models/bluerov/urdf/bluerov.xacro use_front_camera:=$(arg use_front_camera) use_vertical_camera:=$(arg use_vertical_camera)" />
         <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model $(arg vehicle_name) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)" />
         <node name="esc_commander" pkg="fav_sim" type="esc_commander" output="screen" />
         <node name="mixer" pkg="fav_sim" type="simple_mixer" output="screen">
               <rosparam command="load" file="$(find fav_sim)/config/mixer_default.yaml" />
         </node>
      </group>
   </launch>


.. image:: /res/images/gazebo_camera.png

.. hint:: Feel free to modify the camera parameters in :file:`fav/fav_sim/models/camera/urdf/camera_macro.xacro`, if you feel like you need to.

AprilTag Models
===============

In :file:`fav_sim/models/sdf_models` there are many AprilTag models. 128 models in total, to be precise. IDs 0 to 90 are used for the tags on the floor.

Modify Gazebo Worlds
====================

In general, you have two options to get your models into a Gazebo world. Either you use predefined world files like the already mentioned :file:`tank_with_tags.world` and specify the models you want to include.

.. code-block:: xml
   :linenos:
   :emphasize-lines: 28-30

   <?xml version="1.0"?>
   <sdf version="1.6">
      <world name="base">
         <scene>
               <shadows>0</shadows>
         </scene>
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
         <physics type="ode">
               <max_step_size>0.00400</max_step_size>
               <real_time_update_rate>250.0</real_time_update_rate>
         </physics>
         <include>
               <uri>model://apriltag_tank</uri>
         </include>
      </world>
   </sdf>

Or you spawn models during runtime (like it is done for the BlueROV model for example). The :file:`gazebo_ros` package provides functionality for this.

Simply start the node in your launch file and pass it the path to the model's sdf-file you want to spawn. You can also define the initial pose.

To spawn the AprilTag with ID 127 you can add the following lines to your launch file.

.. code-block:: xml
   :linenos:

   <node name="choose_arbitrary_name" pkg="gazebo_ros" type="spawn_model"
       args="-sdf -file $(find fav_sim)/models/sdf_models/tag36_11_00127/model.sdf 
             -model tag_127
             -x 0.1 -y 0.3 -z -0.5
             -R 0.9 -P 0.2 -Y 2.4" />

In line 2, we specify the path to the model file we want to include. In line 3 we choose a name shown in Gazebo for the model.

If you have Gazebo already running, you can run the following commands in a terminal

.. code-block:: sh

   SDF_MODEL=$(rospack find fav_sim)/models/sdf_models/tag36_11_00127/model.sdf

This command stores the path to the model file in a environment variable called :code:`SDF_MODEL`. This is just for convenience. We could also directly substitute :code:`$SDF_MODEL` with the path in the next command.

.. code-block:: sh

   rosrun gazebo_ros spawn_model -sdf -file $SDF_MODEL -model tag_127-x0.1 -y0.3 -z-0.5 -R0.9 -P0.2 -Y2.4

This exectues the model spawner.

.. image:: /res/images/gazebo_spawned_tag.png

.. hint::

   Even though we spawned one of the existing AprilTag models in this example, we are clearly not limited to the existing models. Feel free to add arbitrary models you like.

.. hint::

   There is, of course, documentation available on how to create sdf models. See for example the list of `official gazebo tutorials <https://classic.gazebosim.org/tutorials/browse>`__. 

AprilTag Detection
==================

Might be/probably is relevant for many of you. In general, you do not have to worry about the actual detection pipeline so much. In the Lab **we** will start the tag detection and image pipeline for you. In the simulation, the apriltag detection gets started automagically if the corresponding camera is enabled.

A slight difference between simulation and the lab is the camera distortion. Since we do not simulate distortion, no undistortion is needed. And for the lab, well, there we have some serious distortion we have to get rid of. So the node graph will not look exactly the same, but will still give you the same output: the tag detections.

The node graph containing both pipelines, for the vertical and the front camera. For debugging in the lab, it is often useful to look at the tag detection image. For the simulation the detection rate is actually quite good, so it will probably not that useful.

.. image:: /res/images/apriltag_pipeline_gazebo.png


The detection messages, containing the pose of the detected tags expressed in the corresponding camera frame, has the type :code:`AprilTagDetectionArray`. Please visit the `documentation <http://docs.ros.org/en/noetic/api/apriltag_ros/html/msg/AprilTagDetectionArray.html>`__ for the details on the data fields.  

An example how to access the data fields of the tag detections, is provided below. It is assumed, the node is started in the :file:`bluerov` namespace.

.. code-block:: python

   #!/usr/bin/env python
   import rospy  # this is the python interface for ROS
   from apriltag_ros.msg import AprilTagDetectionArray
   from geometry_msgs.msg import Pose


   class Node():
      def __init__(self):
         rospy.init_node("example_node")
         self.detection_sub = rospy.Subscriber('vertical_camera/tag_detections',
                                                AprilTagDetectionArray,
                                                self.on_detections)

      def on_detections(self, msg: AprilTagDetectionArray):
         rospy.loginfo(f'Detected {len(msg.detections)} tags.')
         for detection in msg.detections:
               # there are only multiple ids, if we specify tag bundles.
               # since we only have standalone tags, only a single id will be in
               # the id array.
               tag_id = detection.id[0]
               # same for the size as for the id
               tag_size = detection.size[0]
               frame = detection.pose.header.frame_id

               # yeah, many .pose...
               pose: Pose = detection.pose.pose.pose
               p = pose.position
               q = pose.orientation
               rospy.loginfo(
                  f'Tag {tag_id} with size {tag_size:.4f} relative to {frame}: \n'
                  f'pos: {p.x:.2f} | {p.y:.2f} | {p.z:.2f}\n'
                  f'q: {q.w:.3f} | {q.x:.3f} | {q.y:.3f} | {q.z:.3f}')

      def run(self):
         rospy.spin()


   def main():
      node = Node()
      node.run()


   if __name__ == "__main__":
      main()


An example for the tag configuration is given in :file:`fav_sim/config/tags_front_camera.yaml` and :file:`fav_sim/config/tags_vertical_camera.yaml`.

.. hint:: 

   It is better to not modify these configuration files, but to create new ones and tell the launch file about it:

   .. code-block:: sh
      
      roslaunch fav_sim simulation.launch use_vertical_camera:=true tag_file_vertical_camera:=/my/custom/path/to/some/tags.yaml

.. note:: 

   For the lab session just send us your custom :file:`.yaml` file for the tags and we will set up the apriltag pipeline for you. Since our localization relies on the vertical camera, you probably do not want to change the setup for this camera.
