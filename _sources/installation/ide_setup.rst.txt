IDE Setup
#########

For this class we recommend Visual Studio Code. Of course you can use any IDE you like and that works for you. The `ROS Wiki <http://wiki.ros.org/IDEs>`_ lists several options how to use IDEs with ROS.

Installation
============

Ubuntu has :code:`snap` preinstalled so probably the easiest way to install VS Code is to execute

.. code-block:: sh

   sudo snap install --classic code

For more detailed instructions and alternative installation options see the `VS Code Documentation <https://code.visualstudio.com/docs/setup/linux>`_.

VS Code Project Configuration
=============================

If you open a folder with VS Code you can have project specific settings stored in a :file:`.vscode` directory. The basic configuration files are the :file:`settings.json` and the :file:`extensions.json`. There might be some other fies present in a :file:`.vscode` directory depending on the extensions installed.

We provide a basic version of both the above mentioned files.

#. Create the :file:`.vscode` directory

   .. code-block:: sh

      mkdir ~/fav/catkin_ws/.vscode && cd ~/fav/catkin_ws/.vscode


#. Get the files

   .. code-block:: sh

      wget https://gist.githubusercontent.com/lennartalff/d530ffca7affc6340a4d58c360548d5d/raw/c186a94331f4d70ed9af54ecd4e5a4bfeb9fc07e/extensions.json
      wget https://gist.githubusercontent.com/lennartalff/d530ffca7affc6340a4d58c360548d5d/raw/c186a94331f4d70ed9af54ecd4e5a4bfeb9fc07e/settings.json

These files contain the following content:

.. raw:: html

   <script src="https://gist.github.com/lennartalff/d530ffca7affc6340a4d58c360548d5d.js"></script>

Open The Catkin Workspace 
=========================

Open the :file:`~/fav/catkin_ws` directory in VS Code:

.. image:: /res/images/vscode_open_folder.png


If you followed the instructions in the previous section and created the :file:`.vscode` directory and put the :file:`settings.json` and :file:`extensions.json` into it, you probably get a message prompt informing you that there are extension recommendations. These recommendations are based on the :file:`extensions.json`. Install the extensions:

.. image:: /res/images/vscode_recommended_extensions.png

If you did not get the prompt you can install the recommendations by switching to the extensions tab, enter :code:`@recommended` and hit the cloud icon in the **WORKSPACE RECOMMENDATIONS** section:

.. image:: /res/images/vscode_install_recommended.png

Most likely you will be asked to reload the window to apply the changes. You can also manually reload the window with :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`P` and enter :code:`reload`:

.. image:: /res/images/vscode_reload_window.png

.. note:: Sometimes it might not be enough to reload the window. So if you want to be super safe just restart VS Code completly.

Useful Hints
============

Integrated Terminal
*******************

If the integrated terminal of VS Code is not visible by default you can activate it by :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`´` or via the menu bar:

.. image:: /res/images/vscode_view_terminal.png

The integrated terminal has two tabs that are probably of most interest for you:

* The **TERMINAL** tab with the terminal itself, which is as you would expect a quite ordinary terminal. You can open new terminals by clicking the plus symbol and switch between the terminal via the dropdown menu. Or you can split the terminal to have multiple terminals next to each other.

* The **PROBLEMS** tab in which problems concerning your open source code are shown.

.. image:: /res/images/vscode_terminal_problems.png

In the **PROBLEMS** tab you can apply a filter to only show problems for the currently active files. The following image shows problems of several kinds. For example a syntax error due to a missing colon in line 11 or a rather cosmetic problem in line 62 that exceeds the specified line length of 80 characters.

.. image:: /res/images/vscode_problems_tab.png

Formatting
**********

There are tools out there to help you keep your code neat and clean, called formatters. In the provided :file:`settings.json` file a formatter has been already selected. To apply formatting on your currently active file hit :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`I`.

.. note:: Formatting can only be applied if your code is syntactically correct.
