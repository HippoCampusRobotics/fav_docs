FAQ
###

How to Compute Time Differences in ROS?
=======================================

Very Short Answer
*****************

.. code-block:: python
   :linenos:

   ...
   t_now = self.get_clock().now()
   dt_as_duration_object = t_now - self.t_previous
   dt = dt_as_duration_object.nanoseconds * 1e-9
   ...
   self.t_previous = t_now

Detailed Answer
***************

Let's consider a control node as example scenario.

.. code-block:: python
   :linenos:

   class MyController(Node):

       self.__init__(self):
           super().__init__(node_name='my_node')
           self.t_previous = self.get_clock().now()
           ...
       
       def compute_control_output(self, ...):
           t_now = self.get_clock().now()
           dt_as_duration_object = t_now - self.t_previous
           dt = dt_as_duration_object = dt_as_duration_object.nanoseconds * 1e-9
           ...
           self.t_previous = t_now
           ...

When we ask ROS for the current time, we do get the current time as a ``Time`` object instead of a floating point/decimal number.
When we compute the difference between two time objects as in line 10, the result becomes a ``Duration`` object.
There are technical reasons why it is implemented this way in ROS.
Nonetheless, for our controller we need the duration as a ``float``.
The conversion is performed in line 11.

Why Don't We Use the Time Library?
**********************************

In python there is the standard module ``time``, which provides as with the ``time.time()`` function to get the current time based on the computer's clock as ``float``.
The obvious way to work with time would be to use this module.
Especially, since we saw that the ROS library requires us to do somewhat more work to get the time difference in a format that we need.

The main reason why we do **not** use ``time`` nonetheless is that ROS hides the effective time source from us. 
Often, the time source is based on the wall clock which should be synchronous to our computer's system time.
Hence, if a second passes in the real world, both the ``time`` module and ROS will tell us, that this very same time span passed.
But if we do a simulation, we do not care for the real-world time, i.e. wall time, but only for the time that passes by in the simulation. 
ROS can automatically handle this for us.
Thus, the code displayed above will stay the same.
With the ``time`` module there would be no way to implement something that considers the time passed in our simulation environment.
Hence the time stamps and duration will not be consistent and will be plainly wrong if the simulation does not run exactly at real time.
