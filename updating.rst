.. _updating:

Updating
########

Usually we anounce updates relevant for the class.
Thus, you do not have to check for updates proactively.

#. Update the simulation packages

   .. code-block:: console

      $ cd ~/fav/ros2_underlay/src/hippo_simulation \
      && git pull origin && git checkout fav-23/24

#. Update the core packages

   .. code-block:: console

      $ cd ~/fav/ros2_underlay/src/hippo_core \
      && git pull origin && git checkout fav-23/24

#. Update the ``fav`` package

   .. code-block:: console

   $ cd ~/fav/ros2/src/fav \
   && git pull origin

#. Rebuild the workspace

   .. code-block:: console

      $ build_underlay

#. Make sure everything is sourced

   .. code-block:: console

      $ source ~/.zshrc
