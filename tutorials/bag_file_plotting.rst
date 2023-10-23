Bag File Plotting
################# 

.. todo::

   This section is still work-in-progress.
   It will be completed soon.
   You can ignore it for now.

In the first part, we will learn how to record data to a bag file and how access this data for plotting and further evaluation.
In the second part, we will go through a suggested workflow to plot this data in a thesis-/paper-/report-ready format that makes the reviewer happy, because we provide him with beautiful vector graphics with appropriate font sizes, scaling and line thicknesses.

Record a Bag File
===================

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

.. code-block:: sh

   ros2 bag record -a -x '(.*)camera(.*)' -o my_bag_file

Let's have a detailed look at the command

-a
   Record everything.
   This way we do not have to specify individual topics.

-x ``'(.*)camera(.*)'``
   Exclude certain topics.
   We can either write them out directly or we can use regular expressions.
   The ``'(.*)camera(.*)'`` matches any topic name containing the substring ``camera``.
   This way we avoid the most likely unncessary subscription of camera related topics, including camera images.


Extract Data From a Bag File
============================

Create Beatiful Plots in LaTex
==============================
