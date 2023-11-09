Real-Time Plotting
##################

Plotting data is crucial for analyzing (and debugging...) your setup.
A very simple tool is `rqt_plot <http://wiki.ros.org/rqt_plot>`_. 
However, :code:`rqt_plot` is excrutiatingly annoying to use. Downsides are: you can only plot data over time and it's very hard to configure, not ideal at all!

Therefore, we highly recommend you use `plotjuggler <https://github.com/facontidavide/PlotJuggler>`__. 

To install :code:`plotjuggler`, run

.. code-block:: sh

   sudo apt install ros-$ROS_DISTRO-plotjuggler-ros

Now, we can start plotjuggler

.. code-block:: sh

      ros2 run plotjuggler plotjuggler


Getting Data In There
*********************

.. image:: /res/images/plotjuggler_start.png

#. Make sure the **ROS2 topic subscriber** is selected
#. Choose an appropriate buffer length (seconds). Only data received during this timespan is plotted. Usually something between 30s and 60s. Set it to whatever you need.
#. Click **Start**

.. image:: /res/images/plotjuggler_topic_selection.png

#. We need to select the topics we actually want to subscribe to. Subscribing everything might cause massive network traffic and CPU load.
#. Confirm

.. image:: /res/images/plotjuggler_time_series_list.png

#. A list with our selected topics appears (if they are actually published).
#. If we have many topics, we might want to filter the list to find the data fields we want to plot.

Plotting is done by simply dragging the desired data field and dropping it on the plot area.

.. raw:: html

   <div>
     <video autoplay loop muted style="width:100%;">
      <source src="../_static/videos/plotjuggler_drag_and_drop.mp4" type="video/mp4">
      test
     </video>
   </div>

We did it! |partying_face|

Adding Splits
*************
Extremely useful feature!

.. raw:: html

   <div>
     <video autoplay loop muted style="width:100%;">
      <source src="../_static/videos/plotjuggler_splits.mp4" type="video/mp4">
      test
     </video>
   </div>

Parametric Plots
****************

We can do parametric plots (i.e. x-y-plots instead of the x-axis being the time) by using the right mouse button instead of the left one when dragging the selected data fields onto the plot.

Multiple Tabs
*************

.. image:: /res/images/plotjuggler_tabs.png

Saving and Loading the Layout
*****************************

We do not want to do all the configuration stuff over and over again if we have configured a neat plotting setup, do we?

.. image:: /res/images/plotjuggler_layout.png

.. _plotjuggler-bag-file:

Plotting Bag File Data
**********************

Instead of streaming live data, we can also plot data stored in a bag file (see :ref:`bag-file-plotting`).
Really helpful to see what we actually have recorded (this sometimes differs from what we *intended* to record).

.. image:: /res/images/plotjuggler_bagfile.png

Scaling
*******

There are only rare cases for which auto scaling the y axis is a good idea. So we better make sure we set some reasonable scaling ourselves.

.. image:: /res/images/plotjuggler_edit_curves.png

#. Open the context menu with the right mouse button on the plot we want to configure and edit the curves.

.. image:: /res/images/plotjuggler_scaling.png

#. Choose appropriate minimum and maximum values.
#. Confirm.

Removing Lines
**************

The same menu as before:

.. image:: /res/images/plotjuggler_remove.png
