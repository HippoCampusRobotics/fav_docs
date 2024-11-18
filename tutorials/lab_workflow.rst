Lab Workflow
============

Time for experiments!

.. .. image:: /res/images/peanut_butter_jelly.gif
..    :align: center
..    :width: 10%

.. figure:: /res/images/lab_work1.jpg
    :width: 80%
   

Checklist
*********

#. Do not **ever** start the simulation/gazebo while you are connected to our network.
   Otherwise we will have some unpleasant topic collision between the simulated vehicle and the real BlueROV in our lab.

#. It's best if one of you brings a **fully prepared software setup on your own laptop**.
   The real experiment will take less computing power since you are not running the simulation environment.
   If you cannot do this, let us know **in time** and we will prepare a laptop for you to use. 

#. The setup can include a prepared settings file (xml) for :code:`plot_juggler`. We reccomend to edit the xml-file as a preparation for the labs changing the namespace from the simulated BlueROV ``bluerov00`` to the real one ``bluerov01``

.. note::

   The lab is **not** the simulation. Hello, Captain Obvious 😱.
   Hence, whereever we have a ``use_sim_time:=true`` in our terminal commands, we have to make sure we write ``use_sim_time:=false`` while we are in the lab.

At the Lab
**********

#. The first time you're at the lab, we will give you an overview of the robot and its sensors.
   We will take care of connecting the battery and starting all ROS nodes and software running on the vehicle.
   Additionally, we will provide you with push buttons to enable/disable the BlueROV's thrusters.
   It can be quite handy to be able to quickly switch off the thrusters.

#. Before lifting the BlueROV into the water, we will do a vaccuum test together. After the test, **do not forget to close the vents!**

   .. image:: /res/images/vaccuum_test.gif
      :width: 40%
   .. image:: /res/images/vent_plugs.jpg
      :width: 40%

   Vaccuum test and vent caps saying "OK".


#. We check if your network setup is working properly.

#. If you want to launch any nodes, you will need to make sure to use the right namespace, since the real robot's name is different: ``vehicle_name:=bluerov01``

#. We have a big TV screen at the lab. You can (and should!) use it (we have HDMI and DisplayPort) for plotting data etc. so that everyone can see.

#. If you record data via :code:`ros2 bag record`, do not include camera images (or, if you *really* need camera data, include just one topic).
   Camera images can overload the network.
   You can exclude topics matching regular expressions with :code:`-x`.

   For example

   .. code-block:: console

      $ ros2 bag record -a -x '(.*)camera(.*)' -o my_bag_file
   
   should exclude all topics containing :file:`camera` and write the data to a file called ``my_bag_file``.
   You can find more information in our section :ref:`record-bag-file`.


#. And after all, if nothing is working, a few debugging tips: 
   
   * check the node graph to see if everything is connected (in the correct way)
   * check the topic monitor (Is the data plausible?)
   * check the debug messages published (of course you have prepared some :) )
   * ask us to check our setup (we make mistakes, too)

.. note:: 

   **Feel free to ask questions at any time!** 
   Let us know if you need any equipment (measuring tape, scale, weights, stick, ...). If you're looking for something we might not have at hand immediately, please ask beforehand.


.. figure:: /res/images/lab_work2.jpg
    :width: 80%
