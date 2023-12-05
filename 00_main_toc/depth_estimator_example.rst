Kalman Filter: Depth-Estimator Example
======================================

We use a Jupyter notebook. 
So instead of having a 'live' simulation in ROS, we run a simulation with a predefined time span, all at once. We also have to include some code that simulates the real system. In ROS, these data would come from either Gazebo or the real world.


Download the Code
#################

Do not clone this repository into the ROS2 workdpace.


.. code-block:: console

   $ cd && git clone https://github.com/FormulasAndVehicles/depth_estimator_notebook.git

.. code-block:: console

   $ cd depth_estimator_notebook

Create a virtual environment

.. code-block:: console

   $ python3 -m venv venv

Source the environment

.. code-block:: console
   
   $ source venv/bin/activate

Install the dependencies

.. code-block:: console

   $ python3 -m pip install -r requirements.txt

Open the Notebook
#################

Start VSCode and open the repository's directory. 
VSCode probablly suggests to install the Jupyter extension. If not, you can install it manually.

You can then open the file :file:`ekf_example.ipynb` and run the notebook.
