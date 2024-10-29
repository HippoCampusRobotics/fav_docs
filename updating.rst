.. _updating:

Updating
########

Usually, we anounce updates relevant for the class.
Thus, you do not need to check for updates proactively.

#. Update the system packages

   .. code-block:: console

      $ sudo apt update && sudo apt upgrade

#. Update the ``fav`` package

   .. code-block:: console

      $ cd ~/fav/ros2/src/fav \
      && git pull origin

#. Rebuild the normal workspace

   .. code-block:: console

      $ build_ros
   
#. Make sure everything is sourced

   .. code-block:: console

      $ source ~/.zshrc
