Install ROS
############

The `ROS documentation <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html>`_ provides a complete installation guide. The following instructions are a summary of their article.

.. attention:: 

   Since Ubuntu 18.04, Canonical decided it was a good idea to have unattended updates running in the background. This might lead to error messages similiar to:

   .. code-block:: console

      E: Could not get lock /var/lib/dpkg/lock-frontend - open (11: Resource temporarily unavailable)
      E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
   
   If you encounter them after running anything related to :code:`apt`: no worries. Just wait until the backgroud updates are finished.

.. hint:: We use copy-buttons, i.e. a copy-button appears when you hover over a code-block. This makes copy-pasting easier than manually selecting the code-block.

   .. image:: /res/images/copy-button.png

Preparation
===========

#. Enable universe repository
   
   .. code-block:: console
      
      $ sudo apt install software-properties-common \
      && sudo add-apt-repository universe

#. Add the key

   .. code-block:: console

      $ sudo apt update && sudo apt install curl -y \
      && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#. Add sources

   .. code-block:: console

      $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null




#. Update

   .. warning:: This is critical!
   

   .. code-block:: console

      $ sudo apt update && sudo apt upgrade -y

Installation
============

#. Install ROS

   .. code-block:: console

      $ sudo apt install ros-jazzy-desktop-full

#. Install build dependencies

   .. code-block:: console

      $ sudo apt install ros-dev-tools python3-pip

#. Set up :code:`rosdep`

   .. code-block:: console

      $ sudo rosdep init && rosdep update

#. Source the ROS installation

   .. code-block:: console

      $ echo 'source /opt/ros/jazzy/setup.zsh' >> ~/.zshrc

#. Apply the changes to your :file:`.zshrc`

   Either close and reopen all open terminals or run the following command in **all** terminals:

   .. code-block:: console

      $ . ~/.zshrc

A Brief Test
============

Managed to follow the instructions until here? To give you a bit of a good feeling, we try to run the following command

.. code-block:: console

   $ ros2 run turtlesim turtlesim_node

A window appears? Lucky you! If no window appears, try to figure out what went wrong or ask your favorite research associate for help.

You can stop the program with :kbd:`Ctrl` + :kbd:`C`.
