Assignment 2 - Localization and Control
#######################################

.. attention:: 

   Please update the :code:`fav` repository. 

   .. code-block:: sh

      cd ~/fav/catkin_ws/src/fav && git pull
   
   We have prepared a new launchfile for you to start with: :file:`gazebo_assignment2.launch`. Use this launchfile instead of :file:`gazebo_apriltag_tank_world.launch` in order to include the simulated range sensor.


.. Get the Code
.. ============

.. You will need to update your :code:`bluerov_sim` package, for example by:

.. .. code-block:: sh

..    roscd bluerov_sim && git pull


.. We have prepared a :code:`range_sensor` package that will publish the range measurements in a topic :code:`/ranges`:

.. .. code-block:: sh

..    git clone https://github.com/FormulasAndVehicles/range_sensor.git ~/fav/catkin_ws/src/range_sensor

.. To install missing dependencies:

.. .. code-block:: sh

..    cd ~/fav/catkin_ws && rosdep install --from-paths src --ignore-src -r -y


.. Don't forget to rebuild your catkin workspace after downloading these packages.

.. .. We have made some adjustments to the PX4-Autopilot firmware running on the Flight Control Unit as well, so we need to update this:

.. .. .. code-block:: sh

.. ..    cd ~/fav/fav_PX4-Autopilot && git pull

.. .. And to rebuild the code, execute in the firmware's directory:

.. .. .. code-block:: sh

.. ..    DONT_RUN=1 make clean 
.. ..    DONT_RUN=1 make -j1 px4_sitl gazebo_uuv_bluerov2_heavy

.. .. .. note::

.. ..    If you got an internal compiler error last time you built the firmware, this will probably happen again. Just repeat the build command a few times until it works.

In this assignment, you will implement an algorithm for autonomous navigation within a confined environment. This will include a localization method and a path tracking controller. 

Again, keep in mind that the best controller isn't any good if your position data is rubbish. Set your priorities accordingly.


Range Sensor in Simulation
==========================

We are not actually using simulated camera images and running the AprilTag algorithm in simulation. Rather, we have written a Gazebo Plugin that simulates the range measurements to the four given anchors. This has the benefit of reducing the computational burden.

Range Sensor parameters
-----------------------

This range sensor plugin has a few parameters we encourage you to play with. In the :code:`fav_sim` package you can find the \*.xacro file defining our BlueROV2 model for Gazebo: :code:`models/range_sensor/urdf/range_sensor_params.xacro`.

The range sensor plugin parameters can be found here:

.. code-block:: XML

    <xacro:property name="noise_stddev" value="0.01" />
    <xacro:property name="drop_probability" value="0.02" />
    <xacro:property name="max_detection_distance" value="5.0" />
    <xacro:property name="drop_probability_exponent" value="2.0" />
    <xacro:property name="max_viewing_angle" value="140.0" />
    <xacro:property name="max_fov_angle" value="90.0" />


* **noise_stddev**: We add white, Gaussian noise on the distance measurements, this sets the standard deviation.
* **rangeNoiseStd**: 
* **drop_probability**: The probability a single distance measurement is dropped.
* **max_detection_distance** and **drop_probability_exponent**: We compute an additional probability that a tag measurement will be dropped which increases with increasing distance to the tag. This is independent of the general **drop_probability**.
* **max_viewing_angle**: Angle limit between x-axis of the robot (the camera) and the normal axis of the detected tag.
* **max_fov_angle**: The camera's field of view (fov) angle. Tags outside this cone won't be detected.

We compute this probability according to the following equation, where the **drop_probability_exponent** is n:

.. math::

   p_{\text{drop}}(\text{dist}) = \frac{1}{\text{maxDist}^{n}} \cdot \text{dist}^{n}

You can use these parameters to test your algorithm in different situations and prepare for reality with varying degrees of pessimism.

Range Sensor Position
---------------------

The range sensor's position for the simulation is defined in the :code:`models/bluerov/urdf/bluerov_params.xacro` file (relative to the body frame :code:`base_link`):

.. code-block:: XML
   
    <xacro:property name="range_sensor_position" value="0.2 0 0.1" />
    <xacro:property name="range_sensor_orientation" value="${radians(0)} ${radians(0)} ${radians(0)}" />


This hasn't been measured precisely (one of us looked at the BlueROV2 and squinted their eyes to estimate the front camera's position...) and you will probably want to make sure this position is adaptable in your code, if you define (and use) it somewhere.


.. hint::

   When you implement your localization and problems arise, you might want to turn off the noise to test whether your algorithm is working at all. 

.. note::

   For the experiment, we are reusing four AprilTags that are already used on the floor on the other side of the tank. Therefore, if the BlueROV gets completely lost, you might get false measurements. However, if you keep the four tags on the wall within the camera's field of view, this shouldn't be a problem.

Taking it Further
=================

In the following, we collected a few hints for you. They are supposed to help you dive deeper into the simulation (if you want to).

In general, we encourage you to use our :code:`keyboard_control` node for convenient testing of your localization.
Feel free to use the rest of the Gazebo ground truth data, for example the position, to evaluate your algorithm. 

.. hint:: 

   The :code:`tf.transformations` library helps you deal with quaternions. Check the `API <http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html>`_ .

.. .. hint::

..    You can access IMU data in the topic :code:`bluerov/mavros/imu/data`, this has already been filtered by the FCU's on-board estimator.

.. hint::

   As you have noticed in the first experiment, the real BlueROV2 behaves differently to the simulated one. The fact that we've never done a proper parameter identification doesn't help. Priorities... the simulation works perfectly fine for testing and evaluating your algorithms. 
   
   However, if you want to adjust how the BlueROV2 is behaving in simulation, you can find some parameters in the :file:`bluerov_params.xacro` file, too. You'll be mostly interested in the linear and angular damping, and the buoyancy:

   .. code-block:: XML
      
      <xacro:property name="damping_linear" value="7 7 7" />
      <xacro:property name="damping_angular" value="0.3 0.3 0.3" />
      <xacro:property name="buoyancy_compensation" value="1.001" />
      <xacro:property name="buoyancy_origin" value="0 0 0.005" />


.. .. hint::

..    Similarly to the BlueROV2 sdf file, there's an sdf file for the tank, where the AprilTag models are included. If you're curious about trying other tag positions (or orientations), you can do this here:

..    .. code-block:: XML
      
..       <include>
..          <name>tag_1</name>
..          <uri>model://tag36_11_00000</uri>
..          <pose>0.5 3.35 -0.5 1.57079632679 -0.0 0.0</pose>
..       </include>

.. .. .. attention::

..    The given anchor positions might still change in the real experiment. We'll keep the rectangular shape, but can't guarantee we will get the distances to be exactly what was announced. Make the positions easily adjustable in your code.



Some Final Remarks
==================

In this assignment you will do different things, including implementing a self-localization algorithm for the BlueROV2 and a controller to autonomously navigate inside the tank. Think about your code structure and package structure to ensure (re-)usability.

.. attention::

   Please do not change code (apart from playing with parameters purely for your simulation, of course) in our repositories, namely :code:`fav_sim`. Instead, create your own packages.




