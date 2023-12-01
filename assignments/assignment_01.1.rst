Assignment 1.1
##############

.. todo::

   This section still needs to be finished.

.. attention::

   You will need to update our repositories: :ref:`updating`.

There are two new launch files in the updated ``fav`` package.

simulation_with_tags.launch.py
   Analogue to ``simulation.launch.py`` but does additionally spawn landmark tags used for distance based localization.

simulation_with_tags_and_keyboard_control.launch.py
   Extends the above mentioned launch file by the keyboard control node.
   This might be particularly useful for testing your localization algorithms befor combining it with control nodes.

We can start the simulation with keyboard control with

.. code-block:: console

   $ ros2 launch fav simulation_with_tags_and_keyboard_control.launch.py vehicle_name:=bluerov00

Note the four AprilTags on the opposite side of the vehicle.

The measured distances to these tags are published under the ``ranges`` topic inside the vehicles namespace.
Details about the the message definitions can be queried with

.. code-block:: console

   $ ros2 interface show hippo_msgs/msg/RangeMeasurementArray 
   std_msgs/Header header
      builtin_interfaces/Time stamp
         int32 sec
         uint32 nanosec
      string frame_id
   hippo_msgs/RangeMeasurement[] measurements
      std_msgs/Header header
         builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
         string frame_id
      int32 id
      float64 range

Most relevant is the ``measurements`` array.
Each element of this array consists of a ``id`` field and a ``range`` field.
The ``id`` identifies the anchor/landmark/AprilTag to which the ``range`` (i.e. distance) was measured.
These IDs will be in the range [0;3] but the array of measurements is not guaranteed to be ordered.

If a landmark/AprilTag was not detected, it will not be included in the ``measurements`` array.
Thus, the ``measurements`` array is of variable size.

In Python we can iterate over the measurements with the following loop:

.. code-block:: python
   :linenos:
   
   for i, measurement in enumerate(msg.measurements):
      tag_id = measurement.id
      measured_distance = measurement.range
      self.get_logger().info(
          f'The {i}. element contains the measurement of tag {tag_id} with the '
          f'distance of {measured_distance}m')

.. seealso::

   A similar snippet can be found in the template code we provide for this assignment.

The Distance Sensor
===================

The distance sensor is located at the front camera's position of the BlueROV, as depicted in :ref:`camera-sensors`.
Hence, the measurements are relative to this position.
Usually we consider the center of the vehicle as the robot's position.
It is fine to to the localization for the camera and apply the transformation to the robot's center in a post-processing step.

In the simulation the position of the distance sensor is exactly known and has an offset of ``[0.2, 0.0, 0.1]`` relative to the vehicle's origin.

