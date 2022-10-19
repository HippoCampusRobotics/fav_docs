Lab Workflow
============

Time for experiments!

.. .. image:: /res/images/peanut_butter_jelly.gif
..    :align: center
..    :width: 10%

.. image:: /res/images/lab_work1.jpg
    :width: 80%
    :align: center

Checklist
*********

The following steps can be prepared at home before coming to the lab.

#. Set the :code:`ROS_MASTER_URI` and :code:`ROS_HOSTNAME` environment variables in :file:`~/.bashrc`. You can have both configurations, for your simulation setup and your experiment setup and comment out the one you are not using with :code:`#`.

   .. tabs::

      .. code-tab:: sh Simulation

         export ROS_HOSTNAME="$(hostname --short).local"

         # Your simulation/home setup
         export ROS_MASTER_URI="http://$(hostname --short).local:11311"

         # Your realworld/lab setup
         #export ROS_MASTER_URI="http://hippo-celsius.local:11311"

      .. code-tab:: sh Lab

         export ROS_HOSTNAME="$(hostname --short).local"

         # Your simulation/home setup
         #export ROS_MASTER_URI="http://$(hostname --short).local:11311"

         # Your realworld/lab setup
         export ROS_MASTER_URI="http://hippo-celsius.local:11311"

   .. note:: As always, run :code:`source ~/.bashrc` to apply these changes.

#. Do not start the simulation/gazebo while you are connected to our network. Otherwise we will have some unpleasant topic collision between the simulated vehicle and the real BlueROV in our lab.

#. It's best if one of you brings a **fully prepared software setup on your own laptop**. The real experiment will take less computing power since you are not running the simulation environment. If you cannot do this, let us know **in time** and we will prepare a laptop for you to use. 

#. The setup can include a prepared settings file for :code:`rqt_multiplot`.

At the Lab
**********


#. The first time you're at the lab, we will give you an overview of the robot and its sensors. We will take care of connecting the battery and starting all ROS nodes and software running on the vehicle. Additionally, we will provide you with push buttons to enable/disable the BlueROV's thrusters. It can be quite handy to be able to quickly switch off the thrusters.

#. Before lifting the BlueROV into the water, we will do a vaccuum test together. After the test, **do not forget to close the vents!**

   .. image:: /res/images/vaccuum_test.gif
      :width: 40%
   .. image:: /res/images/vent_plugs.jpg
      :width: 40%

   Vaccuum test and vent caps saying "OK".


#. We check if your network setup is working properly.

#. We have a big TV screen at the lab. You can use it (we have HDMI and DisplayPort) for plotting data etc. so that everyone can see.

#. If you record data via :code:`rosbag`, do not include camera images (or if you need camera data, include just one topic). Camera images can overload the network. You can exclude topics matching regular expressions with :code:`-x`.
   For example

   .. code-block:: sh

      rosbag record -a -x "(.*)camera(.*)"
   
   should exclude all topics containing :file:`camera`.

#. And after all, if nothing is working, a few debugging tips: 
   
   * check the node graph to see if everything is connected (in the correct way)
   * check the topic monitor (Is the data plausible?)
   * check the debug messages published (of course you have prepared some :) )
   * ask us to check our setup (we make mistakes, too)

.. note:: 

   **Feel free to ask questions at any time!** 
   Let us know if you need any equipment (measuring tape, scale, weights, stick, ...). If you're looking for something we might not have at hand immediately, please ask beforehand.


.. image:: /res/images/lab_work2.jpg
    :width: 80%
    :align: center
