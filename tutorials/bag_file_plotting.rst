.. _bag-file-plotting:

Bag File Plotting
################# 

In the first part, we will learn how to record data to a bag file (:ref:`record-bag-file`) and how access this data for plotting and further evaluation (:ref:`extract-bag-file-data`).

In the second part, we will go through a suggested workflow to plot this data in a thesis-/paper-/report-ready format that makes the reviewer happy, because we provide him with beautiful vector graphics with appropriate font sizes, scaling and line thicknesses.
This way, the reader is not distracted and can appreciate the relevant information of the plot.

.. _record-bag-file:

Record a Bag File
=================

Prerequisites
*************

To follow along this tutorial with an identical setup, we suggest the launch setup from assignment 1:

.. code-block:: console

   $ ros2 launch fav simulation.launch.py vehicle_name:=bluerov00

and our control setup

.. code-block:: console

   $ ros2 launch depth_control depth.launch.py vehicle_name:=bluerov00

In general, you could also use any other setup and adapt the instructions where needed.

Recording the Bag
*****************

We use the command line tool ``ros2 bag record`` for recording bag files.
See the `official docs <https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#ros2-bag-record>`__ for more details.
   
In general, we have to specify which topics we want to record and the name of the output file.
The latter is optional and if not specified, the name will be the current date and time.
We encourage you to choose a meainingful file name.
This way it is easier to manage your bag files later on.

We can either choose each topic name manually.
If we are interested in many topics, this might be not that convenient.
Alternatively, we can instruct ``ros2 bag record`` to record *all* topics.
This is very convenient, but might lead to unecessary large bag files, which might get us into trouble.
Furthermore, subscribing to all topics might even overload the network capacity and negatively impact our system.

.. note:: 

   Choose the topics to record wisely.

As a sensible suggestion, we provide you with the following snippet

.. code-block:: console

   $ ros2 bag record -a -x '(.*)camera(.*)' -o my_bag_file

Let's have a detailed look at the command

-a
   Record everything.
   This way we do not have to specify individual topics.

-x ``'(.*)camera(.*)'``
   Exclude certain topics.
   We can either write them out directly or we can use regular expressions.
   The ``'(.*)camera(.*)'`` matches any topic name containing the substring ``camera``.
   This way we avoid the most likely unncessary subscription of camera related topics, including camera images.

The command will print the topics it subscribes to on the screen.
This way we can already check if the topics we wanted to subscribe are actually recorded.

.. code-block:: console

   $ ros2 bag record -a -x '(.*)camera(.*)' -o my_bag_file 

   [INFO] [1699443379.136203027] [rosbag2_recorder]: Press SPACE for pausing/resuming
   [INFO] [1699443379.149421111] [rosbag2_recorder]: Listening for topics...
   [INFO] [1699443379.149427346] [rosbag2_recorder]: Event publisher thread: Starting
   [INFO] [1699443379.165404721] [rosbag2_recorder]: Subscribed to topic '/tf_static'
   [INFO] [1699443379.167117074] [rosbag2_recorder]: Subscribed to topic '/tf'
   [INFO] [1699443379.172027901] [rosbag2_recorder]: Subscribed to topic '/parameter_events'
   [INFO] [1699443379.174665871] [rosbag2_recorder]: Subscribed to topic '/events/write_split'
   [INFO] [1699443379.179409041] [rosbag2_recorder]: Subscribed to topic '/rosout'
   [INFO] [1699443379.186010634] [rosbag2_recorder]: Subscribed to topic '/bluerov00/odometry'
   [INFO] [1699443379.186489321] [rosbag2_recorder]: Recording...
   [INFO] [1699443379.837613418] [rosbag2_recorder]: Subscribed to topic '/bluerov00/thruster_command'
   [INFO] [1699443379.953955551] [rosbag2_recorder]: Subscribed to topic '/bluerov00/thrusts'
   [INFO] [1699443379.962161809] [rosbag2_recorder]: Subscribed to topic '/clock'
   [INFO] [1699443379.971717827] [rosbag2_recorder]: Subscribed to topic '/bluerov00/pressure'
   [INFO] [1699443379.978513537] [rosbag2_recorder]: Subscribed to topic '/bluerov00/ground_truth/pose'
   [INFO] [1699443379.981857609] [rosbag2_recorder]: Subscribed to topic '/bluerov00/ground_truth/odometry'
   [INFO] [1699443379.988602954] [rosbag2_recorder]: Subscribed to topic '/bluerov00/imu'
   [INFO] [1699443379.991957938] [rosbag2_recorder]: Subscribed to topic '/bluerov00/esc_rpm'
   [INFO] [1699443379.997513827] [rosbag2_recorder]: Subscribed to topic '/bluerov00/angular_velocity'
   [INFO] [1699443380.002118454] [rosbag2_recorder]: Subscribed to topic '/bluerov00/acceleration'
   [INFO] [1699443380.255532039] [rosbag2_recorder]: Subscribed to topic '/bluerov00/thrust_setpoint'
   [INFO] [1699443380.264418114] [rosbag2_recorder]: Subscribed to topic '/bluerov00/depth_setpoint'
   [INFO] [1699443380.379486377] [rosbag2_recorder]: Subscribed to topic '/bluerov00/depth'
   [INFO] [1699443444.550188742] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
   [INFO] [1699443444.577895499] [rosbag2_recorder]: Event publisher thread: Exiting
   [INFO] [1699443444.578708119] [rosbag2_recorder]: Recording stopped
   [INFO] [1699443444.604525312] [rosbag2_recorder]: Recording stopped

.. note::

   The recording is stopped with :kbd:`Ctrl` + :kbd:`C`.

Inspecting the Bag File
***********************


To see important information of the recorded bag file, we use

.. code-block:: console

   $ ros2 bag info my_bag_file/

Note the ``Count`` entry at the end of the topic lines.
This way you can verify that messages have been recorded.

.. code-block:: console

   $ ros2 bag info my_bag_file/

   Files:             my_bag_file_0.mcap
   Bag size:          16.5 MiB
   Storage id:        mcap
   Duration:          65.352s
   Start:             Nov  8 2023 11:36:19.187 (1699443379.187)
   End:               Nov  8 2023 11:37:24.539 (1699443444.539)
   Messages:          103159
   Topic information: Topic: /bluerov00/thrust_setpoint | Type: hippo_msgs/msg/ActuatorSetpoint | Count: 5224 | Serialization Format: cdr
                      Topic: /bluerov00/depth_setpoint | Type: hippo_msgs/msg/Float64Stamped | Count: 3214 | Serialization Format: cdr
                      Topic: /bluerov00/acceleration | Type: geometry_msgs/msg/Vector3Stamped | Count: 3146 | Serialization Format: cdr
                      Topic: /bluerov00/angular_velocity | Type: hippo_msgs/msg/AngularVelocity | Count: 15730 | Serialization Format: cdr
                      Topic: /bluerov00/esc_rpm | Type: hippo_msgs/msg/EscRpms | Count: 15731 | Serialization Format: cdr
                      Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: 15738 | Serialization Format: cdr
                      Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                      Topic: /bluerov00/ground_truth/odometry | Type: nav_msgs/msg/Odometry | Count: 3157 | Serialization Format: cdr
                      Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 3186 | Serialization Format: cdr
                      Topic: /bluerov00/thruster_command | Type: hippo_msgs/msg/ActuatorControls | Count: 5467 | Serialization Format: cdr
                      Topic: /bluerov00/odometry | Type: nav_msgs/msg/Odometry | Count: 3186 | Serialization Format: cdr
                      Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                      Topic: /bluerov00/pressure | Type: sensor_msgs/msg/FluidPressure | Count: 5246 | Serialization Format: cdr
                      Topic: /bluerov00/thrusts | Type: hippo_msgs/msg/ThrusterForces | Count: 15740 | Serialization Format: cdr
                      Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                      Topic: /bluerov00/ground_truth/pose | Type: geometry_msgs/msg/PoseStamped | Count: 3157 | Serialization Format: cdr
                      Topic: /bluerov00/depth | Type: hippo_msgs/msg/DepthStamped | Count: 5213 | Serialization Format: cdr
                      Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 23 | Serialization Format: cdr
                      Topic: /bluerov00/imu | Type: sensor_msgs/msg/Imu | Count: 0 | Serialization Format: cdr

.. note::

   Nothing is more frustrating than recording bag files while having some great experiment-time, but when we get home, we realize that the bag file is empty and no data has been recorded!
   **Make sure**, this is not happening to you.

Often it is quite useful to have a look on the recorded data **even during the lab session**.
Luckily, this is rather easy to accomplish with ``plotjuggler`` as described :ref:`here <plotjuggler-bag-file>`.

.. _extract-bag-file-data:

Extract Data From a Bag File
============================

.. todo::

   This section is still work-in-progress.
   It will be completed soon.
   You can ignore it for now.

*"Dude, we already have such a nice plot of our bag file data in plotjuggler, what else could we possibly want?"* |br|
True that, but a few aspects to motivate this section include but are not limited to

* the data we record is not the data we want to plot (let's say we want to display the distance between to robots but the bag file *only* contains the position for both of the).
* we want to create a plot based on multiple subequent runs of the same experiment and overlay/combine them.
* we need to annotate our plot or highlight some sections
* we want our plot to of visually sufficient quality to support the information we would like to present sufficiently well (lines that are too thin, axis labels that are hardly readable, etc. all distract from the information that should be conveyed by the plot).

Workflow
********

We suggest the following workflow.

#. Read the bag file from within a python program.
#. Do the required data processing in python with ``numpy``. ``numpy`` is the de-facto standard library for number crunching in python.
#. Preview the plots directly in python with ``matplotlib``. ``matplotlib`` is the de-facto standard plotting library in python. With its submodule ``pyplot`` it should feel very familiar to the way we would plot in Matlab.
#. Export the data we want to finally plot as ``.csv`` file.
#. Look forward in joyful anticipation of the creation of the final plot in ``LaTex``.

Load the Bag File
*****************

.. note::

   We will use the simulation setup based on assignment 1 for demonstration purposes.
   The general workflow should be understandable without having done this assignment and can be easily adapted for other scenarios.

We write a python program and use the ``rosbag2_py`` modue to read the bag file.

We want to create the following directory structure:

.. code-block:: console

   ~/fav/bag_evaluation
   ├── main.py
   ├── my_bag_file
   │   ├── metadata.yaml
   │   └── my_bag_file_0.mcap
   └── reader.py

.. note::

   We can create this directory and the ``main.py`` and ``reader.py`` directly inside VSCode!
   The ``my_bag_file`` directory is the bag file directory created via ``ros2 bag record``.

We copy-paste the following code into ``reader.py``

.. code-block:: python
   :linenos:
   :caption: reader.py

   import rosbag2_py
   from rclpy.serialization import deserialize_message
   from rosidl_runtime_py.utilities import get_message


   class Reader():

       def __init__(self, bag_file: str):
           self.bag_file = bag_file

       def _read_topic(self, selected_topic: str):
           reader = rosbag2_py.SequentialReader()
           reader.open(
               rosbag2_py.StorageOptions(uri=self.bag_file, storage_id="mcap"),
               rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                           output_serialization_format="cdr"),
           )
           topic_types = reader.get_all_topics_and_types()

           def typename(topic_name):
               for topic_type in topic_types:
                   if topic_type.name == topic_name:
                       return topic_type.type
               raise ValueError(f'topic {topic_name} not in bag')

           while reader.has_next():
               topic, data, timestamp = reader.read_next()
               if topic != selected_topic:
                   continue
               msg_type = get_message(typename(topic))
               msg = deserialize_message(data, msg_type)
               yield topic, msg, timestamp
           del reader

       def get_data(self, topic: str):
           return [[x[1], x[2]] for x in self._read_topic(topic)]


We do not care for the details of the reader implementation for now.
It simply provides us with the functionality to read data from the bag file.

To have a minimal working example, we paste the follwing snippet into ``main.py``

.. code-block:: python
   :linenos:
   :caption: main.py

   #!/usr/bin/env python3

   from reader import Reader

   def main():
       reader = Reader('my_bag_file')
       data = reader.get_data('/bluerov00/depth_setpoint')

       for (msg, time_received) in data:
           print(f'Message: {msg}\ntime_received: {time_received}')


   if __name__ == '__main__':
       main()

In a terminal (an integrated one of VSCode or open a new one with :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`) we make sure we are in the same directory as the ``main.py`` file

.. code-block:: console

   $ cd ~/fav/bag_evaluation

and run the ``main.py`` script

.. code-block:: console

   $ python3 main.py
   ...
   Message: hippo_msgs.msg.Float64Stamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1699443444, nanosec=495445714), frame_id=''), data=-0.6)
   time_received: 1699443444495922280
   Message: hippo_msgs.msg.Float64Stamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1699443444, nanosec=515611158), frame_id=''), data=-0.6)
   time_received: 1699443444516101056
   Message: hippo_msgs.msg.Float64Stamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1699443444, nanosec=535362893), frame_id=''), data=-0.6)
   time_received: 1699443444536112988

We see the list of messages printed to the screen.




Create Beatiful Plots in LaTex
==============================
