Linux Terminal
##############

During the following setup procedure you will have to use the terminal. This section gives you some hints on how to use the terminal effectively. Feel free to skip this part and come back later, as soon as you have installed/configured Ubuntu in the next chapter. Or continue reading, to be already prepared when you arrive there.

For those of you not quite that familiar with Linux and/or the terminal and the command line, this section will give you some useful hints.

Open a Terminal
===============
Open a terminal with :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`.

In many cases it will be necessary/convenient to have multiple terminal sessions. So its recommendable to install :code:`terminator`, to have multiple terminals multiplexed in a single window. Install it with the following command:

.. code-block:: sh

   sudo apt install terminator

Cook Delicious Copy-Pasta
=========================

Of course you can copy&paste in terminals via keyboard shortcuts. But you will additionally need to use the :kbd:`Shift` key.

copy
   :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`C`

paste
   :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`V`

Autocompletion
==============

If you start to enter commands you can hit :kbd:`Tab` to trigger automatic completion. If there are multiple possibilities, you need to hit :kbd:`Tab` a second time to show you the options.

Really useful. Use it. Believe me. It's awesome.

Quit Running Commands/Programs
==============================

:kbd:`Ctrl` + :kbd:`C`. You have probably already assumed :kbd:`Ctrl` + :kbd:`C` has to be good for something if you can not use it to copy stuff.

Entering Passwords
==================

If you execute commands with :code:`sudo` to gain root privileges, you will be asked to enter your password. Do **not** worry, if no characters appear while you are entering it. That is quite normal. Just enter it and hit :kbd:`Enter`.

Use The History
===============

By pressing the up/down arrow keys, you can navigate through your recently entered commands. Or you can search your command history by using the shortcut :kbd:`Ctrl` + :kbd:`R` to search in your command history.

Further Readings
================

Have a look at the `"Command line for beginners" <https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>`_ Ubuntu tutorial.