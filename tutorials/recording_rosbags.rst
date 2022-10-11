Recording Data Using Bag Files
##############################

You can record data conveniently in bags. More information on bag files can be found `here <http://wiki.ros.org/Bags>`_.
The standard tool to record and play back bag files is `rosbag <http://wiki.ros.org/rosbag>`_.

Typically, you want to use this tool from the command-line. 

.. attention:: 

   Please check the command-line documentation here: `http://wiki.ros.org/rosbag/Commandline <http://wiki.ros.org/rosbag/Commandline>`_!

Recording bag files
===================

You can record data using :code:`rosbag record`. This subscribes to topics and writes a bag file with the contents of all messages published on those topics.

To record specific topics, use

.. code-block:: sh

   rosbag record topic1 topic2

High bandwidth messages, such as camera images, can overload the network. For this reason, do not record camera images. If you really need camera data, include just a single topic. Due to image processing, our setup includes a lot of camera image topics. Recording all of these topics **will** overload the network. 

You can exclude topics matching regular expressions with :code:`-x`.
To record all but the camera topics, execute

.. code-block:: sh

   rosbag record -a -x "(.*)camera(.*)"

where :code:`(.*)` stands for an arbitrary number of arbitrary characters. This means that any topic including the substring :code:`camera` will not be recorded.  


.. attention:: 

   While a temporary binary file is created when :code:`rosbag record` is called, the final file isn't written until stopping the recording using :kbd:`Ctrl` + :kbd:`C`.
   Always make sure to properly end your recording!

.. image:: /res/images/rosbag_record.gif
   :width: 60%
   :align: center


   
Checking and playing  back bag files
====================================

Once you have recorded a bag file, you might want to check its contents.
Often times, you realize after the experiments that some topic is missing, due to e.g. typos.
Note that we cannot give you additional lab time if you forgot to record all data, so make sure to check while still in the lab! 
It can be useful to record a test-bag in the beginning for this purpose.

Use :code:`rosbag info` to display a human-readable summary of the contents of the bag files.

Additionally, :code:`rqt` has a tool to analyze bag files. You can find it under :file:`Plugins/Logging/Bags`.

.. image:: /res/images/rqt_bag_tool.gif
   :width: 80%
   :align: center

.. image:: /res/images/rqt_bag_tool_example.png
    :width: 80%
    :align: center


After loading your bag file, you get a list of recorded topics. Moreover, you can see at what times during the recording messages were actually received (displayed in blue here).



Finally, you can replay bag files using :code:`rosbag play`. 

.. attention:: 

   **Never replay rosbag files while the real vehicle is running**. ROS cannot differentiate between messages send live and messages replayed at that moment. While the time stamp may be different, replaying e.g. actuator signals will lead to unexpected behaviour!

