Assignment 3
############

.. note:: 

   Remember to regularly update your local repositories as described in the previous section. In case we make major changes, we'll send a slack reminder to update.


Get the Code
============

You will need to update your :code:`bluerov_sim` package, for example by:

.. code-block:: sh

   roscd bluerov_sim && git pull


We have prepared a :code:`range_sensor` package that will publish the range measurements in a topic :code:`/ranges`:

.. code-block:: sh

   git clone https://github.com/FormulasAndVehicles/range_sensor.git ~/fav/catkin_ws/src/range_sensor

To install missing dependencies:

.. code-block:: sh

   cd ~/fav/catkin_ws && rosdep install --from-paths src --ignore-src -r -y


Don't forget to rebuild your catkin workspace after downloading these packages.

.. We have made some adjustments to the PX4-Autopilot firmware running on the Flight Control Unit as well, so we need to update this:

.. .. code-block:: sh

..    cd ~/fav/fav_PX4-Autopilot && git pull

.. And to rebuild the code, execute in the firmware's directory:

.. .. code-block:: sh

..    DONT_RUN=1 make clean 
..    DONT_RUN=1 make -j1 px4_sitl gazebo_uuv_bluerov2_heavy

.. .. note::

..    If you got an internal compiler error last time you built the firmware, this will probably happen again. Just repeat the build command a few times until it works.


Range Sensor in Simulation
==========================

We are not actually using simulated camera images and running the AprilTag algorithm in simulation. Rather, we have written a Gazebo Plugin that simulates the range measurements to the four given anchors. This has the benefit of reducing the computational burden.

The range sensor plugin has a few parameters we encourage you to play with. In the :code:`bluerov_sim` package you can find the \*.sdf file defining our BlueROV2 model for Gazebo: :code:`models/uuv_bluerov2_heavy/uuv_bluerov2_heavy.sdf`

The range sensor plugin parameters can be found here:

.. code-block:: XML

   <plugin name="range_sensor_plugin" filename="libgazebo_range_sensor_plugin.so">
      <robotNamespace />
      <pubRate>7.0</pubRate>
      <rangeNoiseStd>0.1</rangeNoiseStd>
      <fovCamera>100</fovCamera>
      <viewingAngle>140</viewingAngle>
      <dropProb>0.05</dropProb>
      <maxDetectionDist>4.0</maxDetectionDist>
      <distDropProbExponent>2.0</distDropProbExponent>
   </plugin>

* **pubRate**: The publishing rate for the range measurements.
* **rangeNoiseStd**: We add white, Gaussian noise on the distance measurements, this sets the standard deviation.
* **fovCamera**: The camera's field of view (fov) angle.
* **viewingAngle**: The viewing angle at which a tag is still detectable.
* **dropProb**: The probability a single distance measurement is dropped.
* **maxDetectionDist** and **distDropProbExponent**: We compute an additional probability that a tag measurement will be dropped which increases with increasing distance to the tag. This is independent of the general **dropProb**.

We compute this probability according to the following equation, where the **distDropProbExponent** is n:

.. math::

   p_{\text{drop}}(\text{dist}) = \frac{1}{\text{maxDist}^{n}} \cdot \text{dist}^{n}

You can use these parameters to test your algorithm in different situations and prepare for reality with varying degrees of pessimism.

.. hint::

   When you implement your localization and problems arise, you might want to turn off the noise to test whether your algorithm is working at all. 

.. hint::

   The range sensor's position for the simulation is defined in the :code:`uuv_bluerov2_heavy.sdf` file as well (relative to the body frame :code:`base_link`):

   .. code-block:: XML
      
      <link name="range_sensor_link">
         <pose>0.2 0 0.1 0 0 0</pose>
         ...
      </link>

   This hasn't been measured precisely (one of us looked at the BlueROV2 and squinted their eyes...) and you will probably want to make sure this position is adaptable in your code, if you define it somewhere.


Taking it Further
=================

In the following, we collected a few hints for you. They are supposed to help you dive deeper into the simulation.

In general, we encourage you to use our :code:`keyboard_control` node for convenient testing.

.. note::

   Feel free to use the rest of the Gazebo ground truth data, for example the position, to evaluate your localization algorithm. Keep in mind that velocities are given in the world frame "map". Our docs section :ref:`next_steps/resources:Coordinate transformations` includes some pointers for where to look.

.. hint:: 

   The :code:`tf.transformations` library helps you deal with quaternions. Check the `API <http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html>`_ .

.. hint::

   You can access IMU data in the topic :code:`mavros/imu/data`, this has already been filtered by PX4's own estimator.

.. hint::

   As you have noticed in the first experiment, the real BlueROV2 behaves differently to the simulated one. The fact that we've never done a proper parameter identification doesn't help. Priorities... the simulation works perfectly fine for testing and evaluating your algorithms. 
   
   However, if you want to adjust how the BlueROV2 is behaving in simulation, you can find some parameters in the sdf file, too. You'll be mostly interested in our :code:`uuv_plugin`, where you can change the linear and angular damping, and the buoyancy:

   .. code-block:: XML
      
      <plugin name="uuv_plugin" filename="libgazebo_uuv_plugin.so">
            ...
            <dampingLinear>7 7 7</dampingLinear>
            <dampingAngular>0.3 0.3 0.3</dampingAngular>
            ...
            <buoyancy>
                <link_name>base_link</link_name>
                <origin>0 0 0.01</origin>
                <compensation>1.001</compensation>
                <height_scale_limit>0.05</height_scale_limit>
            </buoyancy>
        </plugin>


.. hint::

   Similarly to the BlueROV2 sdf file, there's an sdf file for the tank, where the AprilTag models are included. If you're curious about trying other tag positions (or orientations), you can do this here:

   .. code-block:: XML
      
      <include>
         <name>tag_1</name>
         <uri>model://tag36_11_00000</uri>
         <pose>0.5 3.35 -0.5 1.57079632679 -0.0 0.0</pose>
      </include>

.. .. attention::

..    The given anchor positions might still change in the real experiment. We'll keep the rectangular shape, but can't guarantee we will get the distances to be exactly what was announced. Make the positions easily adjustable in your code.



Some Final Remarks
==================

In this assignment you will do different things, including implementing a self-localization algorithm for the BlueROV2 and a controller to autonomously navigate inside the tank. Think about your code structure and package structure to ensure (re-)usability.

.. attention::

   Please do not change code (apart from playing with parameters purely for your simulation, of course) in our repositories, namely :code:`bluerov_sim` and :code:`range_sensor`. There are some adjustments missing for the experiment that we are working hard on, so you will have to be able to pull our uploaded code from Github.
   Instead, create your own packages.




