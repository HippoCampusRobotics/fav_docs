Troubleshooting
###############

What You Should Try First
=========================

Make sure your workspace is built

.. code-block:: console

   $ build_ros

and make sure your workspace is sourced after you added new packages/nodes!

.. code-block:: console

   $ source ~/.zshrc

I cannot run my node!
=====================

Problem
*******

You get a python exceptions with the last line of

.. code-block:: console

   Traceback (most recent call last):
   ...

being the following error

.. code-block:: console

   OSError: [Errno 8] Exec format error: <path/to/your/node>

Solution
********

You have forgotten to add

.. code-block:: python

   #!/usr/bin/env python3

as first line of your node.
Better add it now.


Problem
*******

You get the output

.. code-block:: console

   No executable found

Solution
********

Double check that the node exists, gets installed (``CMakeLists.txt``, see :ref:`tutorials/ros_package:Run A Node`), and is executable.
If you have forgotten to make it executable (most likely reason of the before mentioned causes), run 

.. code-block:: console

   $ chmod +x <PATH_TO_YOUR_NODE>

