ROS Launch Setup
################

.. attention::

   The following tutorial **is not meant as a step-by-step solution for the first assignment.** These are just **toy examples** to demonstrate how to use ROS and interact with the simulated BlueROV *in an easy to follow manner*. Therefore, we do not claim that these code snippets are complete and we use some funny names at times. Please do **not** copy-paste them.


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

Since now we have a :file:`launch` directory, we have to tell our build system to *install* it.

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

we get an error message that the launch file could not be found.
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
    
We observe our node has been started.
And this time, the execution does not terminate by itself.
Stop everything by hitting :kbd:`Ctrl` + :kbd:`C`.

What comes next? 
================
A lot!

* "pushing" nodes "into namespaces"
* including other launch files
* using launch *arguments*

The python-based launch workflow in ROS2 may appear quite complex and cumbersome when launch files get more complicated than our previous toy example.
Do not feel discouraged by this and do not worry if you do not manage to understand everything immediately!
You will get used to to it, step by step each time you work with it.

So, why are we using lanch files, you might ask.
Because it greatly simplifies launching our setups.
Write the launch file once and profit every time we start any setup.
Trust me, you will start things **many** times.

Pushing Nodes into Namespaces
=============================

Why do we care about namespaces?
We want to avoid topic name collisions. 
Just imagine we have more than one node publishing a debug topic, calling it ``debug``.
Or what about having multiple robots?
We can easly imagine operating two BlueROVs at the same time.
How can we distinguish between topics associated with the first and the second robot?
Having different source code with manually changed topic names for both robots? 
Does not sound like a way anyone would like to go.
Here, ROS namespaces come to the rescue!
Simply pushing nodes to so-called namespaces can avoid all these problems.

We have a great overview on the topic of namespaces in :ref:`tutorials/ros_names_and_namespaces:Names and Namespaces`.
What we recommend is as a guideline:

* Use namespaces where appropriate (in the course of this class: most likely **everywhere**).
* Never use *global* topic names if you do not have a specific reason to do so.
* yeah, that's actually it...

Let's illustrate that with the help of our :file:`setpoint_publisher.py` we created in the previous section.
We created the publisher with

.. code-block:: python

   self.create_publisher(ActuatorSetpoint, 'thrust_setpoint', 1)

Topic names starting with ``/`` are *global*.
Hence, the topic name stays always exactly what we defined, no matter what namespaces the node is in, or what the node's name is.
|br|
"*But dude, I do not see a leading* ``/`` *here* ".
|br|
True that. Thus, we have specified a *relative* topic (and not a *global* one).
This means that the topic name will by resolved at runtime: prepending all nested namespaces of our node. 
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
For the first method, we hand over a ``namespace`` parameter when creating the ``Node`` action.

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

we can observe that the node now publishes under the corresponding namespace.
Isn't this just awesome?
We do not have to touch our actual source code at all and are still able to configure our node!

So now let us talk about the second method, which might look like it requires a bit more work.
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

Did you recognize that we *nested* two namespaces this way? 
Since we are still defining a namespace in ``Node``, and additionally push the node to another namespace with  the name ``PushRosNamespace``, we end up with a topic name that concatenates these namespaces.
We do not need this for now, but we might want to keep this in mind.
It might become useful in some situations.

This second approach is more flexible because we are not limited to ``Node`` actions that are pushed to our desired namespace.
We can even push whole launch files to namespaces, since including launch files is done by using actions.
The action of including other launch files can be put inside the ``GroupAction``, just like any other action.

This brings us to our next topic.

Using Launch Arguments
======================
We have seen that we can configure our node in some way (i.e. prepending a namespace to topic names) without touching its source code.
The next step is to configure our launch file without the need of changing it.
We do not want to *hardcode* the namespace.
We can imagine that we would like to use the same launch setup, i.e. starting the same nodes, for different vehicles with different vehicle *names*.
To differentiate between the vehicles, we would like to use the vehicle name as a namespace name.
Without launch arguments this would mean that we would either have to change our launch file constantly between different launches or we would need almost identical launch files with just different values for the namespace for each setup.
Both approaches are not that attractive.

Instead, we would like to pass the namespace via the command line during runtime.
We need two things for that.
First, we *declare* the argument we would like to pass via the ``DeclareLaunchArgument`` action and add this action to our launch description.
Second, we *access* the value of this argument via ``LaunchConfiguration`` and use it as parameter for ``PushRosNamespace`` instead of hardcoding the value.

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
In general, We can pass arguments with ``<argument_name>:=<argument_value>``.
Thus, our launch command becomes

.. code-block:: sh

   ros2 launch awesome_package setpoint.launch.py vehicle_name:=my_vehicle_name

Verify that the topic name gets changed accordingly to how you define the ``vehicle_name`` argument in the command line.


Including Launch Files
======================

Okay, okay. Admittedly, we introduced a lot of new and maybe not that easy to understand concepts regarding launch files in ROS.
But stay with us for this very last subsection.

It is not only possible to combine *sets* of nodes in a launch file, but also to combine launch files themselves.
Remember the launch file we used to verify that our workspace setup is working?

.. code-block:: sh

   # do not run this now
   ros2 launch fav simulation.launch.py vehicle_name:=bluerov00

Let us include this launch file in our awesome ``setpoint.launch.py`` launch file.
We will need ``PythonLaunchDescriptionSource`` and ``IncludeLaunchDescription`` to accomplish this.

.. code-block:: python
   :linenos:
   :caption: ~/fav/ros2/src/awesome_package/launch/setpoint.launch.py
   :emphasize-lines: 8, 10, 27-33

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
Therefore, we need to *pass* it to our launch file as we have seen above.

Also, we will run across an argument called ``use_sim_time`` quite often.
For the *simulation*, we hardcoded it to ``true``.
Hence, it is not necessary to manually set this argument in our example launch file.
This parameter controls the *time source* of a node.
If set to true, nodes will automatically subscribe to a special topic which provides the current time.
In this case, the actual time of the computer (wall time) is ignored.
Instead, a simulated time, starting at 0 each time you restart the simulation, is used.
This is obviously very useful for simulations. 

Depending on the performance of our computers, the simulation might be slower than real-time.
If your computer is very fast, you might even simulate *faster* than real-time!
By using the simulated time as time source, the simulation (gazebo) can control how fast time passes by from the perspective of the nodes.

As a simple rule, the value should always be ``true`` for simulation setups and always be ``false`` for real world experiments.