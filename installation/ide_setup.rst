IDE Setup
#########
.. role:: strike
   :class: strike

:strike:`For this class we recommend Visual Studio Code`.
We encourage you to use the IDE/editor of your choice.
People are different and so are their needs and desires.
Person A might prefer ``neovim``, while Person B likes **CLion**, whereas Person C is doing everything in **windows-notepad**.
Nonetheless, we think VSCode is very beginner friendly and capable at the same time.
Thus, for all of you not already having a preferred editor we present a basic setup for VSCode.

This does **not** mean, you could not switch to ``neovim`` later on...

What is an IDE
==============

Simply speaking, an IDE is a more advanced text editor, with certain capabilites to simplify writing of program code.
If you have experience with Matlab: the program that you have used to write your Matlab code is an example of an IDE.

.. note::

   VSCode is probably the tool you will spent by far the most time with.
   You will probably have it opened all the time.
   You can create files and directories in it and will do all your programming in it too.

Installation
============

Ubuntu has :code:`snap` preinstalled so probably the easiest way to install VS Code is to execute

.. code-block:: console

   $ sudo snap install --classic code

For more detailed instructions and alternative installation options see the `VS Code Documentation <https://code.visualstudio.com/docs/setup/linux>`_.

VS Code Project Configuration
=============================

If you open a folder with VS Code you can have project specific settings stored in a :file:`.vscode` directory.
The basic configuration files are the :file:`settings.json` and the :file:`extensions.json`.
There might be some other fies present in a :file:`.vscode` directory depending on the extensions installed.

We provide a basic version of both the above mentioned files.

#. Create the :file:`.vscode` directory

   .. code-block:: console

      $ mkdir ~/fav/ros2/.vscode \
      && cd ~/fav/ros2/.vscode

#. Open an editor with the file name that we want to create

   .. code-block:: console

      $ gedit ~/fav/ros2/.vscode/extensions.json

   and paste the following content

   .. code-block:: json
      :caption: extensions.json

      {
          "recommendations": [
               // generate docstring snippets for python
            "njpwerner.autodocstring",
               // python language support
            "ms-python.python",
            "eeyore.yapf",
               // cpp language support
            "ms-vscode.cpptools",
               // yaml language support
            "redhat.vscode-yaml",
               // ROS extension
            "ms-iot.vscode-ros",
               // xml language support
            "redhat.vscode-xml",
               // syntax support for cmake files
            "twxs.cmake",
               // generate doxygen comments
            "cschlosser.doxdocgen"
          ]
      }

#. Save the file with :kbd:`Ctrl` + :kbd:`S` and close the editor.

#. Repeat the steps for ``settings.json``

   .. code-block:: console

      $ gedit ~/fav/ros2/.vscode/settings.json

   and paste the following content

   .. code-block:: json
      :caption: settings.json

      {
           "[python]": {
               "editor.formatOnSaveMode": "file",
               "editor.formatOnSave": true,
               "editor.defaultFormatter": "eeyore.yapf",
           },
           "yapf.args": ["--style", "{based_on_style: pep8, indent_width: 4, column_limit: 80}"],
           "python.analysis.completeFunctionParens": true,
           "clang-format.language.cpp.enable": true,
           "autoDocstring.docstringFormat": "google",
           "files.insertFinalNewline": false,
           "yaml.format.enable": true,
           "yaml.validate": true,
           // use google style per default
           "C_Cpp.clang_format_fallbackStyle": "Google",
           // never fall back to tag parser
           "C_Cpp.intelliSenseEngineFallback": "disabled",
           "C_Cpp.codeAnalysis.clangTidy.enabled": true,
           // use compile_commands.json specified in c_cpp_properties.json
           "C_Cpp.codeAnalysis.clangTidy.useBuildPath": true
       }

   Again, save with :kbd:`Ctrl` + :kbd:`S` and close the window.

.. note:: 

   In some occasions ``yapf`` does not get installed automatically by VSCode.
   Better to be safe than sorry, so make sure it is installed by executing

   .. code-block:: console

      $ python3 -m pip install yapf

Open the Workspace
==================

.. code-block:: console

   $ cd ~/fav/ros2 \
   && code .

If you followed the instructions in the previous section and created the :file:`.vscode` directory and put the :file:`settings.json` and :file:`extensions.json` into it, you probably get a message prompt informing you that there are extension recommendations.
These recommendations are based on the :file:`extensions.json`. Install the extensions:

.. image:: /res/images/vscode_recommended_extensions.png

If you did not get the prompt you can install the recommendations by switching to the extensions tab, enter :code:`@recommended` and hit the cloud icon in the **WORKSPACE RECOMMENDATIONS** section:

.. image:: /res/images/vscode_install_recommended.png

Most likely you will be asked to reload the window to apply the changes. You can also manually reload the window with :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`P` and enter :code:`reload`:

.. image:: /res/images/vscode_reload_window.png

.. note:: Sometimes it might not be enough to reload the window. So if you want to be super safe just restart VS Code completely.

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

In the **PROBLEMS** tab you can apply a filter to only show problems for the currently active files. The following image shows problems of several kinds. For example, a syntax error due to a missing colon in line 11 or a rather cosmetic problem in line 62 that exceeds the specified line length of 80 characters.

.. image:: /res/images/vscode_problems_tab.png

Formatting
**********

There are tools out there to help you keep your code neat and clean, called formatters. In the provided :file:`settings.json` file a formatter has been already selected. To apply formatting on your currently active file, hit :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`I`.

.. note:: Formatting can only be applied if your code is syntactically correct.
