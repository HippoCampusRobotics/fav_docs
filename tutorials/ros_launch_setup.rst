ROS Launch Setup
################

.. attention::
   The following tutorial **is not meant as a step-by-step solution for the first assignment.
   These are just **toy examples** to demonstrate how to use ROS and interact with the simulated BlueROV in an *easy to follow manner*.
   Therefore, we do not claim that these code snippets are complete and we use some funny names at times.
   Please do **not** copy-paste them.
   Use reasonable names for reasonable things.

Prerequisites
=============

We assume, we have the package created in the previous tutorial.
Hence, the package structure should resemble this

.. code-block:: sh

   ~/fav/ros2/src/awesome_package
   ├── nodes
   │   └── setpoint_publisher.py
   ├── CMakeLists.txt
   └── package.xml

Let's Start
===========

We create a new directory called :file:`launch`

.. code-block:: sh

   cd ~/fav/ros2/src/awesome_package
   mkdir launch

and create a launch file

.. code-block:: sh

   touch launch/setpoint.launch.py

and start with a very minimal version of a launch file

.. code-block:: python
   :linenos:

   from launch import LaunchDescription

   def generate_launch_description() -> LaunchDescription:
      launch_description = LaunchDescription()
      return launch_description

Since now we a :file:`launch` directory, we have to tell our build system to *install* it.

Open :file:`CMakeLists.txt` and add the following lines right before the :code:`ament_package()` call.

.. code-block:: cmake

   install(
     DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}
   )

These give the build system instructions to install all the directories following ``DIRECTORY``.

.. note::
   We only have to add this once.
   Even if we add more launch files.
   The whole directory gets installed by this instruction.

If we try to run the launch file with

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py

we get an error message, that the launch file could not be found.
No reason to trust anyone blindly.
Try it out yourself!

What did we forget?
We did not rebuild our workspace.
The instructions in :file:`CMakeLists.txt` are only executed when we build the workspace with

.. code-block:: sh

   build_ros

Now, try it again.
The launch command above should succeed. 
Since it only consists of boilerplate code, not much will happen.
Time to add some functionality.

Launch a Node
=============

In the previous tutorial we have started our awesome :file:`setpoint_publisher.py` via ``ros2 run``.
Let's see how we would accomplish this with our launch file.

.. code-block:: python
   :linenos:
   :caption: setpoint.launch.py

   from launch_ros.actions import Node
   from launch import LaunchDescription


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()

       node = Node(executable='setpoint_publisher.py', package='awesome_package')
       launch_description.add_action(node)

       return launch_description

.. note::
   We do not have to rebuild anything. Just make sure the file has been saved.
   Rebuilding is only required when we add new files.

We again start the launch file

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py

and see the following output

.. code-block:: sh

   [INFO] [launch]: All log files can be found ...
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [setpoint_publisher.py-1]: process started with pid [4991]
    
We observe, our node has been started.
And this time the execution does not terminate by itself.
Stop everything by hitting :kbd:`Ctrl` + :kbd:`C`.

What comes next? 
================
A lot!

* pushing nodes into namespaces
* including other launch files
* using launch arguments

The python based launch workflow in ROS2 may appear quite complex and cumbersome if launch files get more complicated than our previous toy example.
Do not feel discouraged by this and do not worry, if you do not manage to understand everything immediately.
You will get used to to it, step by step each time you work with it.

So, why are we using lanch files, you might ask.
Because it greatly simplifies launching our setups.
Write the launch file once and profit the many times we start any setup.
Trust me, you will start things **many** times.

Pushing Nodes into Namespaces
=============================

Why do we care about namespaces?
We want to avoid topic name collisions. 
Just imagine we have more than one node publishing a debug topic calling it ``debug``.
Or what about having multiple robots?
We can easly imagine operating two BlueROVs at the same time.
How can we distinguish between topics by the first and the second robot?
Having different source code with manually changed topic names for both robots? 
Does not sound like a way anyone would like to go.
ROS namespaces come to the rescue.
Just pushing nodes to namespaces might avoid all these problems.

We have a great overview on the namespace topic in :ref:`tutorials/ros_names_and_namespaces:Names and Namespaces`.
What we recommend is as a guideline:

* Use namespaces where appropriate (in the course of this lecture most likely **everywhere**).
* Never use global topic names if you do not have a specific reason to do so.
* yeah, that's actually it...

Let's illustrate that with the help of our :file:`setpoint_publisher.py` we created in the previous section.
We created the publisher with

.. code-block:: python

   self.create_publisher(ActuatorSetpoint, 'thrust_setpoint', 1)

Topic names starting with ``/`` are global.
Hence, the topic stays always exactly what we defined, no matter what the namespaces are the node is in or what the node name is.
|br|
"*But dude, we do not have a leading* ``/``".
|br|
True that. Thus, we have specified a relative topic.
So every namespace our node is in will be prepended to the actually resolved topic name.
We can quickly see this by pushing our node to different namespaces and check the resulting topic name with ``ros2 topic list``.

.. tab-set::

   .. tab-item:: Without Namespace
      
      .. code-block:: sh

         ros2 run awesome_package setpoint_publisher.py

      ``ros2 topic list`` will show the topic name :file:`/thrust_setpoint`.

   .. tab-item:: With Namespace

      .. code-block:: sh

         ros2 run awesome_package setpoint_publisher.py --ros-args -r __ns:=/my_namespace

      ``ros2 topic list`` will show the topic name :file:`/my_namespace/thrust_setpoint`.
      You can also try others namespaces if you like.
      Just note that namespaces have to start with a leading ``/``.

"*But didn't we want do this inside a launch file? We are in the launch file section!*"
|br|
Okay, we have two ways to push nodes into namespaces in launch files.
For the first method we hand over a ``namespace`` parameter when creating the ``Node`` action.

.. code-block:: python
   :caption: setpoint.launch.py
   :linenos:
   :emphasize-lines: 10

   from launch_ros.actions import Node
   from launch import LaunchDescription


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()

       node = Node(executable='setpoint_publisher.py',
                   package='awesome_package',
                   namespace='my_namespace')
       launch_description.add_action(node)

       return launch_description

When we start the launch file with

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py

We can observe that the node publishes now under the corresponding namespace.
Isn't this just awesome?
We do not have to touch our actual source code at all and are still able to configure our node!

So now let us talk about the second method, which might look it requires a bit more work.
But at the same time it is more powerful.
We make use of ``GroupAction`` and ``PushRosNamespace``.
The ``GroupAction`` is just a *container* for actions.
Our ``Node`` is an action, so we will put it inside the ``GroupAction``.
``PushRosNamespace`` is a special action, that pushes all other actions inside the same group into the defined namespace.

.. code-block:: python
   :linenos:
   :caption: ~/fav/ros2/src/awesomepackage/launch/setpoint.launch.py

   from launch_ros.actions import Node, PushRosNamespace

   from launch import LaunchDescription
   from launch.actions import GroupAction


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()

       node = Node(executable='setpoint_publisher.py',
                   package='awesome_package',
                   namespace='my_namespace')
       group = GroupAction([
           PushRosNamespace('pushed_to_this_namespace'),
           node,
       ])
       launch_description.add_action(group)

       return launch_description

Instead of directly adding the ``Node`` action to our launch description, we add the node to the ``GroupAction`` which in turn is then the action added to the launch description.
When starting this launch setup, we get the following result

.. asciinema:: /res/asciinema/ros2_topic_list_nested_namespace.cast
   :speed: 2
   :start-at: 1
   :idle-time-limit: 1
   :poster: npt:0:01

Did you recognize, that we have just nested namespaces? 
Since we are still defining a namespace in ``Node`` and additionally push the node to another namespace with ``PushRosNamespace``, we and up with a topic name that concatenates these namespaces.
We do not need this for now, but we might want to keep this in mind, it might become useful in some situations.

This second approach is more flexible because we are not limited to ``Node`` actions that are pushed to our desired namespace.
We can even push whole launch files to namespaces, since including launch files is done by actions, that can be put inside the ``GroupAction``, as well.

This brings us to our next topic.

Using Launch Arguments
======================
We have seen that we can configure our node in some way (i.e. prepending a namespace to topic names) without touching its source code.
The next step is to configure our launch file without the need of changing it.
We do not want to hardcode the namespace.
We can imagine that we would like to use the same launch setup, i.e. starting the same nodes, for different vehicles with different vehicles.
Without launch arguments this would mean that we would either have to change our launch file constantly between different launches or we would need almost identical launch files with just different values for the namespace for each setup.
Both approaches are not that attractive.

Instead we would like to pass the namespace via the command line during runtime.
We need two things for that.
First, we *declare* the argument we would like to pass via the ``DeclareLaunchArgument`` action and add this action to our launch description.
Second, we access the value of this argument via ``LaunchConfiguration`` and use it as parameter for ``PushRosNamespace`` instead of hardcoding the value.

.. code-block:: python
   :linenos:
   :caption: ~/fav/ros2/src/awesome_package/launch/setpoint.launch.py
   :emphasize-lines: 4-5,11-12, 16

   from launch_ros.actions import Node, PushRosNamespace

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, GroupAction
   from launch.substitutions import LaunchConfiguration


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()

       arg = DeclareLaunchArgument('vehicle_name')
       launch_description.add_action(arg)

       node = Node(executable='setpoint_publisher.py', package='awesome_package')
       group = GroupAction([
           PushRosNamespace(LaunchConfiguration('vehicle_name')),
           node,
       ])
       launch_description.add_action(group)

       return launch_description

If we just start our setup with the usual

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py

We will get an error message

.. code-block:: sh

   [ERROR] [launch]: Caught exception in launch (see debug for traceback): Included launch description missing required argument 'vehicle_name' (description: 'no description given'), given: []

The launch system complains that we do not have provided our recently declared ``vehicle_name`` argument.
We can pass arguments in general with ``<argument_name>:=<argument_value>``.
Thus, our launch command becomes

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py vehicle_name:=my_vehicle_name

Verify that the topic name gets changed accordingly to how you define the ``vehicle_name`` argument in the command line.


Including Launch Files
======================

Okay, okay. Admittedly, we introduced a lot of new and maybe not that easy to understand concepts regarding launch files in ROS.
But stay with us for this very last subsection.
It is not only possible to combine sets of *nodes* in a launch file, but also combine launch files themselves.
Remember the launch file we used to verify our workspace setup is working?

.. code-block:: sh

   # do not run this now
   ros2 launch fav simulation.launch.py vehicle_name:=bluerov00

Let us include this launch file in our awesome ``setpoint.launch.py`` launch file.
We will need ``PythonLaunchDescriptionSource`` and ``IncludeLaunchDescription`` to accomplish this.

.. code-block:: python
   :linenos:
   :caption: ~/fav/ros2/src/awesome_package/include/setpoint.launch.py
   :emphasize-lines: 27-33

   from ament_index_python.packages import get_package_share_path
   from launch_ros.actions import Node, PushRosNamespace

   from launch import LaunchDescription
   from launch.actions import (
       DeclareLaunchArgument,
       GroupAction,
       IncludeLaunchDescription,
   )
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()

       arg = DeclareLaunchArgument('vehicle_name')
       launch_description.add_action(arg)

       node = Node(executable='setpoint_publisher.py', package='awesome_package')
       group = GroupAction([
           PushRosNamespace(LaunchConfiguration('vehicle_name')),
           node,
       ])
       launch_description.add_action(group)

       package_path = get_package_share_path('fav')
       launch_path = str(package_path / 'launch/simulation.launch.py')
       source = PythonLaunchDescriptionSource(launch_path)
       launch_args = {'vehicle_name': LaunchConfiguration('vehicle_name')}
       action = IncludeLaunchDescription(source,
                                         launch_arguments=launch_args.items())
       launch_description.add_action(action)

       return launch_description



Are you wondering what ``launch_arguments`` in line 32 is needed for?
This is required because the included launch file declares launch arguments as well.
If we do not provide it with the arguments that it declares, it will complain about it.
Usually we always use the ``vehicle_name`` parameter as namespace for all vehicle related nodes.

To conveniently find out what arguments are declared by a launch file or in any of its included launch files, we can pass ``-s`` to the launch command.
We can inspect the launch arugments declared by the launch file we included in our ``setpoint.launch.py``, we run

.. code-block:: sh

   ros2 launch fav simulation.launch.py -s

The result will list many arguments.
The only parameter without default value is ``vehicle_name``.
Therefore, we need to pass it our launch file as we have seen above.

Also we will run across a argument called ``use_sim_time`` quite often.
This argument is used to the equally named node parameter.
For the simulation, we harcoded it to ``true``.
Hence, it is not necessary to set this in our example launch file.
This parameter controls the time source of a node.
If set to true, nodes will automatically subscribe to a special topic which provides it with the current time.
In this case the actual time of the computer (wall time) is ignored.
This is useful for simulations. 

Depending on the performance of our computers, the simulation might be slower than real-time.
If our computer is very fast, we might even simulate faster than real-time.
By using the simulated time as time source, the simulation (gazebo) can control how fast time passes by from the perspective of the nodes.
Usually the value should always be ``true`` for simulation setups and always be ``false`` for real world experiments.

.. todo:: from here on follows the non-migrated out-dated ROS1 documentation!

Before We Start
===============

So, before we start to create a super cool launch setup and have some super fancy nodes doing exciting stuff, lets take a step back and have another look on the keyboard-control setup from the setup instructions.
Let us relaunch this setup and open just another terminal to run

.. code-block:: sh

   rqt_graph

Make sure to select Nodes/Topics(all) in the upper left corner and refresh the view.
This should yield a graph like

.. image:: /res/images/keyboard_control_node_graph.png

You can see the different nodes :file:`/bluerov/keyboard`, :file:`/bluerov/mixer` and :file:`/bluerov/esc_commander` (we are not interested in the :file:`gazebo` node and will simply ignore it) inside ellipses and topics inside rectangles.
Since all these nodes live inside the :file:`/bluerov` namespace and use relative topic names, everything has the :file:`/bluerov` prefix (more on this later).

The :file:`esc_commander` node is the interface between the ESCs which drive the thrusters and our ROS domain.
It receives messages of the type :file:`fav_msgs/ThrusterSetpoint` on the :file:`thruster_setpoint` topic.
That should be familiar to all of us from the previous tutorial and our dummy example with the :code:`setpoint_publisher.py`.
The message definition can be looked up in :file:`~/fav/catkin_ws/src/fav/fav_msgs/msg/ThrusterSetpoint.msg` and is:

.. code-block::

   std_msgs/Header header
   float64[8] data

It contains the field :code:`data` that is an :code:`double` array of length 8.
Each entry corresponds to a thruster. 

Now let's imagine the :code:`mixer` node in the above graph would not exist and the :file:`keyboard` node would have to publish messages of type :file:`fav_msgs/ThrusterSetpoint` directly.
This would imply that the :file:`keyboard` node would have to know about the specific thruster configuration of our BlueROV to work.
To move the vehicle forward when pressing :kbd:`W`, the :file:`keyboard` node would need to know that the first four motors are the only ones in horizontal direction and that they are configured in a way that all of them need to spin in positive direction to move the vehicle forward. 

To add a layer of abstraction we have the :file:`mixer` node.
What :file:`keyboard` actually wants to do is to say "the user pressed :kbd:`W`, so move forward (i.e. set a positive value for thrust)" and from there on it is in the :file:`mixer`'s responsibility to translate this to actual setpoints for the specific thrusters that participate in the forward movement of the vehicle.

Basically, we divided a bigger problem into two smaller problems.
In this case, this can be especially handy because also a controller we might program at some later stage does not need to have knowledge of specific thrusters/actuators.
It can directly output commands corresponding to the actuated degrees of freedom of the BlueROV.
And since all degrees of freedom of the vehicle are actuated, we can control all degrees of directly |partying_face|.

Mathematically the :file:`mixer` node computes the following equation:

.. math:: 
   
   \begin{bmatrix}t_0\\\vdots\\t_7\end{bmatrix} = \boldsymbol{M} \begin{bmatrix}\textrm{roll}\\\textrm{pitch}\\\textrm{yaw}\\\textrm{thrust}\\\textrm{vertical thrust}\\\textrm{lateral thrust}\\0\\0\\\end{bmatrix},

where :math:`t_0` to :math:`t_7` are the direct thruster setpoints.

Having Fun with Open-Loop Control
=================================

Let us start where we have left off in the previous :ref:`tutorials/ros_package:ROS Package`  section.

We have a package called :code:`awesome_package`.
And we have a node called :code:`setpoint_publisher.py`.
Since we know about the :file:`mixer` now, we want to use it and have to modify our :file:`setpoint_publisher.py` to publish to the actuation topics instead of publishing directly to the :file:`thruster_setpoint` topic.

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy  # this is the python interface for ROS
   import math  # needed to use the trigonometric functions sin and cos
   from std_msgs.msg import Float64


   class MyFirstNode():
      def __init__(self):
         rospy.init_node("setpoint_publisher")
         self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                      Float64,
                                                      queue_size=1)

      def run(self):
         rate = rospy.Rate(30.0)

         while not rospy.is_shutdown():
               msg = Float64()
               t = rospy.get_time()
               msg.data = 0.5 * math.sin(t)
               self.vertical_thrust_pub.publish(msg)
               rate.sleep()


   def main():
      node = MyFirstNode()
      node.run()


   if __name__ == "__main__":
      main()

We do not created a new package or a new node, so we do not have to rebuild the workspace to apply the changes.
But make sure you have saved the file after making these changes!

Make sure no nodes/launch setups are currently running.
Otherwise stop them with :kbd:`Ctrl` + :kbd:`C` in the corresponding terminals. 

Start the simulation environment

.. code-block:: sh

   roslaunch fav_sim gazebo_apriltag_tank_world.launch

spawn the vehicle

.. code-block:: sh

   roslaunch fav_sim spawn_vehicle.launch

and lastly start our :file:`setpoint_publisher` node:

.. code-block:: sh

   rosrun awesome_package setpoint_publisher.py

And you see... nothing.
This will probably not be the last time things do not work out as expected.
So let us investigate what might be the problem.
Remember :code:`rqt_graph`? Great tool to see how nodes are connected (or not).

The command should yield something like this:

.. image:: /res/images/rqt_graph_setpoint_publisher_fail.png

Make sure to uncheck **Dead sinks** and **Leaf Topics**.
Since the :file:`gazebo` and :file:`gazebo_gui` node are not relevant for our example, we can hide them by inserting :code:`-/gazebo,-/gazebo_gui` in the first text box.
Also make sure **Nodes/Topics (all)** is selected in the upper left corner and refresh the view.

Do you recognize how every node but our poor :file:`setpoint_publisher` lives inside the :file:`/bluerov` box? Now we will interact with namespaces for the first time.
There are three distinct ways to declare topic names.
They are either *global*, *relative*, or *private*. 

In our node we declared the topic name to be *relative*.
But how can we tell? Because there is no leading :file:`/` or :file:`~`. 

.. code-block:: python
   :lineno-start: 9
   :linenos:

   self.vertical_thrust_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)

But what does it mean? It means the effective topic name will not necessarily be exactly :file:`vertical_thrust`.
This depends on the namespace of our node.
Since we did not specify any namespace during :code:`rosrun awesome_package setpoint_publisher.py`, the topic will be resolved as :file:`/vertical_thrust`.
The :file:`mixer` node living inside the :file:`/bluerov` namespace subscribes to the relative topic :file:`vertical_thrust`.
Due to the namespace this will resolve as :file:`/bluerov/vertical_thrust`.
That is the reason why our node is not connected to the :code:`mixer`.

How to fix it, you may ask? We simply push our node into the :file:`/bluerov` namespace.
This makes sense because our node is part of our BlueROV setup.
Another 'fix' would be to prepend :file:`bluerov/` to the topic name of our publisher.
But in this specific scenario I would rather call it botch.
So let us push this node to the right namespace already! Just append :code:`__ns:=bluerov` to the :code:`rosrun` command.

.. code-block:: sh

   rosrun awesome_package setpoint_publisher __ns:=bluerov

This tells our node to live inside the :file:`bluerov` namespace. 

Refresh our view of :code:`rqt_graph` by clicking the refresh button in the upper left corner and you will see, we have a beautifully connected graph!

.. image:: /res/images/rqt_graph_setpoint_publisher_success.png


We can now admire our moving robot in the simulation:

.. image:: /res/images/gazebo_awesome_package.gif

By now we might get worried by the increasing number of needed terminal windows.
Imagine we want to start additional nodes.
Do we really need a separate terminal for each of them? Of course not! Launch files to the rescue!

 
Create A Launch Setup
=====================

Create a new launchfile.
You could name it :file:`setpoint.launch` for example:

.. image:: /res/images/create_launchfile.gif

It could look like this:

.. code-block:: xml
   :linenos:

   <launch>
      <arg name="vehicle_name" default="bluerov" />

      <!-- start the simulation -->
      <include file="$(find fav_sim)/launch/simulation.launch" pass_all_args="true" />

      <group ns="$(arg vehicle_name)">
         <!-- start the setpoint publisher node -->
         <node name="setpoint_publisher" pkg="awesome_package" type="setpoint_publisher.py" />
      </group>
      
      <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" />
   </launch>

Explanation
===========

Let's take a detailed look what we have here.

Arguments
*********

.. code-block:: xml
   :lineno-start: 2
   :linenos:

   <arg name="vehicle_name" default="bluerov" />

Declares an argument named :code:`vehicle_name` and assigns the default value :code:`"bluerov"`.
We will use this argument to set the namespace of the nodes to be launched.
To overwrite this argument without having to modify the launch file, we can simply append :code:`vehicle_name:="A_NEW_VALUE"` to the :code:`roslaunch` command.

Include Files
*************

.. code-block:: xml
   :lineno-start: 5
   :linenos:

   <include file="$(find fav_sim)/launch/simulation.launch" pass_all_args="true" />

We can include other launch files.
It is literally the same as copy pasting the content of the specified file right inside our own launch file.
Furthermore, we have the special syntax :code:`$(find fav_sim)` here.
We do not have to know the full path to the launch file.
We can use :code:`$(find)` to get the path to ros packages.
In case the :code:`pass_all_args` attribute is set to :code:`true`, all arguments in our launch file get passed to the included launch file.
Otherwise this would not be the case.

Groups and Nodes
****************

.. code-block:: xml
   :lineno-start: 7
   :linenos:

   <group ns="$(arg vehicle_name)">
      <!-- launch the motor_command_sender node-->
      <node name="setpoint_publisher" pkg="awesome_package" type="setpoint_publisher.py" />
   </group>

Two things here.
We can declare groups and assign a namespace to everything that is inside this group by settings the :code:`ns` attribute.
To use the arguments we have declared in the launch file or pass in via the command line, we use :code:`$(arg parameter_name)` so in our case :code:`$(arg vehicle_name)`.
To start the :code:`setpoint_publisher` node, we use the :code:`<node>` tag.
The :code:`name` attribute overwrites the node's name set in the sourcode by :code:`rospy.init_node("setpoint_publisher")`. :code:`pkg` is the name of the package where the node is located.
And :code:`type` is the file name of the executable.

.. code-block:: xml
   :lineno-start: 12
   :linenos:

   <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" />

This starts the :code:`rqt_graph` tool directly in our launch setup.
This way we do not have to start it in another terminal to see the nodegraph. 

Launch the Setup
================

So this launch file produces the exact same setup we have created in the section :ref:`tutorials/ros_launch_setup:having fun with open-loop control` before.
The advantage is, we can start it with a single command:

.. code-block:: sh

   roslaunch awesome_package setpoint.launch

Really looks the same, doesn't it? Now stop everything and try to assign the :code:`vehicle_name` parameter from the command line.

.. code-block:: sh

   roslaunch awesome_package setpoint.launch vehicle_name:=klopsi

Everything will still be connected just fine.
The only difference is, that every node is running inside the :file:`/klopsi` namespace.

Taking the Next Step
====================

We can also pass arguments to the launch file that are not declared in the file we are launching directly.
Remember that we set :code:`pass_all_args` to true when including :file:`simulation.launch`? Inside :file:`simulation.launch` the file :file:`spawn_vehicle.launch` is included and all arguments are passed as well. 

.. image:: /res/images/spawn_vehicle.png

There are arguments :code:`x`, :code:`y` and :code:`z` declared for the spawning position of the vehicle and :code:`R`, :code:`P` and :code:`Y` for the orientation.
We can pass arguments all the way down to this launch file.
So we can modify the spawning position of the vehicle by running

.. code-block:: sh

   roslaunch awesome_package setpoint.launch x:=4 z:=-3

Maybe it is necessary to rotate the camera inside gazebo to find the BlueROV in its new position.

Get Sensor Data
===============

At this point we know the basics of actuating the vehicle.
But to know how we want to actuate the vehicle, we might depend on some sensor input. 

The BlueROV has a pressure sensor.
The output of the pressure sensor is published under the :file:`pressure` topic inside the vehicle's namespace.
So by default the topic name will be :file:`/bluerov/pressure`.

Theoretically, we could use the :file:`setpoint_publisher.py` and modify its code to subscribe to the :file:`pressure` topic.
But to keep things modular and separated, we add a new node to the :file:`awesome_package`.
Let's name it :file:`depth_calculator.py`.
You could argue that having a complete program only calculating the depth coordinate of the vehicle from pressure data might seem like a bit overkill.
But let's see the :file:`depth_calculator` as some specific case of a state estimation.
And this can get complex very quickly.
Therefore, it is a good idea to solve separate problems in separate nodes.

.. note:: Keep in mind, you have to make every node executable! See :ref:`tutorials/ros_package:Write A Node`.

The source code might look like this:

.. code-block:: python
   :linenos:

   #!/usr/bin/env python
   import rospy
   from sensor_msgs.msg import FluidPressure
   from std_msgs.msg import Float32


   def pressure_callback(pressure_msg, publisher):
      pascal_per_meter = 1.0e4
      # what kind of pressure data do we get? relative/absolute? What about
      # atmospheric pressure?
      depth = -pressure_msg.fluid_pressure / pascal_per_meter
      depth_msg = Float32()
      depth_msg.data = depth
      publisher.publish(depth_msg)


   def main():
      rospy.init_node("depth_calculator")
      depth_pub = rospy.Publisher("depth", Float32, queue_size=1)
      pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                       pressure_callback, depth_pub)
      rospy.spin()


   if __name__ == "__main__":
      main()

.. hint::
   Confused on how you should know what the structure of a FluidPressure message is and how to access its data? Simply search for "ros fluidpressure" and you will find the `message definition <http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html>`_.
   Message fields are accessed by a dot operator.

We can add this node to our launchfile by adding the following snippet inside the :code:`<group>`` tag:

.. code-block:: xml
   
   <node name="depth_calculator" pkg="awesome_package" type="depth_calculator.py" />

And launch the setup:

.. code-block:: sh

   roslaunch awesome_package setpoint.launch

We can check that the nodes are properly connected in the graph:

.. image:: /res/images/rqt_graph.png

.. note:: Refresh the node graph with the refresh button in the upper left corner to make sure the graph is up-to-date.

And to inspect the data, we can plot it in :code:`rqt_multiplot` 

.. image:: /res/images/depth_multiplot.png

or use the :code:`rqt` topic monitor or simply in the command line:

.. code-block:: sh

   rostopic echo bluerov/depth

We can see that the data is noisy.
And in the real world data is *always* noisy.
But depending on the scenario, there is a wide range of filtering methods available.
One could compute a moving average over the last :math:`n` data points, a very simple software first order lowpass filter or maybe even something more advanced like a Kalman filter or a particle filter.
But the possibilites are of course not limited to those approaches.

The Missing Link
================

So now we have a :code:`depth_calculator` computing the depth of the BluerROV in some way and we have a :code:`setpoint_publisher` publishing vertical thrust values to move the BlueROV.
What about renaming the :code:`depth_calculator` to :code:`depth_estimator` and make the :code:`setpoint_publisher` a :code:`depth_controller`? Maybe a :code:`depth_controller` should subscribe to a setpoint topic as well as to the current depth?
