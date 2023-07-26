Available Sensors
#################

The BlueROV2 has a number of on-board sensors available for you to use.

Inertial Measurement Unit
=========================

The Inertial Measurement Unit (IMU) is on-board the Pixhawk and consists of accelerometer, gyroscope and magnetometer. The accelerometer measures **linear accelerations** and the gyroscope measures **angular velocities**.

The IMU data is processed on the FCU by PX4's estimator. Since the accelerometer also measures accelerations due to gravity, the vehicle's **pitch** and **roll angle** can be estimated well using the IMU data.
For estimating a drone's **yaw angle**, usually a compass (magnetometer) is used. However, since we are deploying the BlueROV2 in a steel tank, the compasses' measurements are not usable.

For accurate yaw estimation, external data has to be fused into PX4's estimator.


Pressure Sensor
===============

A pressure sensor located in the upper enclosure's end cap can measure the (water) pressure. This can be used to estimate the vehicle's height.


Cameras
=======

The BlueROV2 comes with a camera facing forward. This camera's pitch angle is actuated by a servo. We have added a second camera facing downward which is mainly used for our localization using AprilTag markers on the tank floor. 

.. image:: /res/images/bluerov_cameras.png
    :width: 80%
    :align: center




