Updating
########

Usually we anounce updates relevant for the class.
Thus, you do not have to check for updates proactively.

#. Update the simulation packages

   .. code-block:: sh

      cd ~/fav/ros2_underlay/src/hippo_simulation \
      && git pull origin && git checkout fav-23/24

#. Update the core packages

   .. code-block:: sh

      cd ~/fav/ros2_underlay/src/hippo_core \
      && git pull origin && git checkout fav-23/24

#. Rebuild the workspace

   .. code-block:: sh

      build_underlay

#. Make sure everything is sourced

   .. code-block:: sh

      source ~/.zshrc
