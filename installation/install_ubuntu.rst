Install Ubuntu
##############

This section should guide you through the process of installing Ubuntu alongisde Windows on your machine. If you prefer to install Ubuntu inside a Virtual Machine, you can skip this section and continue with :ref:`installation/virtual_machine:Virtual Machine`.

.. attention:: Since we are going to install a new/second operating system on your device, that might be a good opportunity to create a backup just in case anything goes wrong! We are not responsible for any damage or loss of data that might result from any of the following instructions.

.. note:: For those of you who have their windows encrypted with bitlocker you might have to enter your bitlocker key the next time you boot windows after you have installed Ubuntu. You should save your bitlocker key (you can find it `here <https://account.microsoft.com/devices/>`_). Alternatively you can turn off bitlocker encryption.

Prepare Your Disk Partition
===========================


#. Resize your Windows partition. Hit :kbd:`Win` + :kbd:`R` and enter :code:`diskmgmt.msc` and click **OK**

   .. image:: /res/images/windows_execute.png

#. Choose the partition on which you want to install Ubuntu (in most cases this will be :file:`(C:)`.

#. Resize the partition to create unallocated space where we can install Ubuntu. 30GB of free space might be enough for Ubuntu, but we recommend at least 60GB if you can afford it.

   .. image:: /res/images/windows_disk_manager.png

#. The result could look similiar to this:

   .. image:: /res/images/windows_disk_manager_result.png

Create A Bootable USB stick
===========================

#. Download `Ubuntu 22.04 Desktop <https://releases.ubuntu.com/22.04/>`_ from the offical Website.

#. Download `Etcher <https://www.balena.io/etcher/>`_. 

#. Insert an USB stick with at least 4GB of memory.

#. Start Etcher and select the downloaded Ubuntu image and the inserted USB device as target.

   .. image:: /res/images/etcher.png
 
#. Make sure you **really** selected the correct target. Otherwise all your data on the wrongly selected target might get deleted.

#. Click **Flash!**.

Disable Fast Startup
====================

If you want to access your Windows partition from inside Ubuntu, you need to disable Windows' Fast Startup. Follow `these instructions <https://help.uaudio.com/hc/en-us/articles/213195423-How-To-Disable-Fast-Startup-in-Windows-10>`_, if you want to do so. 

Disable Secure Boot
===================

#. Reboot your system after flashing is done and enter your BIOS/UEFI during Bootup. To do so, you have to hit a certain key, depending on your hardware. For Dell commputers you probably need to hit :kbd:`F12` during the DELL splashscreen. For Lenovo, this key is :kbd:`Enter`. This opens a dialog where you can choose to enter your BIOS settings.

#. Find the settings to disable Secure Boot, save your changes and exit the BIOS/UEFI.

Boot From Ubuntu USB Stick
==========================

#. Reboot and select the medium you want to boot from during the splashscreen (again :kbd:`F12` for Dell or :kbd:`Enter` for Lenovo). Now you want to boot from the Ubuntu USB stick.

#. In the Grub menu, choose :code:`Ubuntu` (probably the first option). This will boot Ubuntu from the USB stick after a quick file system check.


Start The Installation Wizard
=============================

#. Once booted, you will get the option to either :code:`Try Ubuntu` or :code:`Install Ubuntu` immediately. If you decide to try Ubuntu before installing, you will find an icon for installing Ubuntu on the desktop. Double click it to launch the installation wizard. If you don't want to try Ubuntu first, you can click on :code:`Install Ubuntu` to start the installation wizard. 

#. We *highly* recommend setting the language to **English**.

#. Choose the option to install additional drivers. It is also a good idea to connect to a nearby WiFi or Ethernet.

   .. image:: /res/images/ubuntu_additional_drivers.png

#. In the last step of the installation, the wizard asks you if you want to install Ubuntu alongside windows because it detects your windows installation and the free disk space we created before. Choose this option and click **Install Now**.

   .. warning:: Do **NOT** choose the **Erase disk and install Ubuntu** option! This will delete your Windows installation!

   .. image:: /res/images/ubuntu_alongside.png

#. Reboot.

You should now have a running Ubuntu version on your laptop! 

This is the time to check that your Windows partition still runs, too. As mentioned at the beginning, you might be asked for the bitlocker key when booting Windows again for the first time.


Some Convenience Features
=========================

.. note:: 

   The following is optional, just some tips/fixes for your convenience.

Now that your dual-boot is set up, you might notice a few changes. 

When booting, you will see the Grub menu. By default, Ubuntu is the first option. If you are mostly using Windows in your daily life, this can become tedious. It's easy to not react quickly enough and to end up booting Ubuntu by accident. Ending up in the seemingly endless rebooting cycle...

While (hopefully!) you will spend a lot of time using Ubuntu this semester, the following explains how to change this back to Windows as the default.

Open the Grub configuration file (for editing this, you need to open with :code:`sudo`)

.. code-block:: sh

   sudo gedit /etc/default/grub

A window should open with contents similar to this:

.. code-block::
   :linenos:
   :emphasize-lines: 6

   # If you change this file, run 'update-grub' afterwards to update
   # /boot/grub/grub.cfg.
   # For full documentation of the options in this file, see:
   #   info -f grub -n 'Simple configuration'

   GRUB_DEFAULT=0
   GRUB_TIMEOUT_STYLE=hidden
   GRUB_TIMEOUT=10
   GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`
   GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
   GRUB_CMDLINE_LINUX=""

   # Uncomment to enable BadRAM filtering, modify to suit your needs
   # This works with Linux (no patch required) and with any kernel that obtains
   # the memory map information from GRUB (GNU Mach, kernel of FreeBSD ...)
   #GRUB_BADRAM="0x01234567,0xfefefefe,0x89abcdef,0xefefefef"

   # Uncomment to disable graphical terminal (grub-pc only)
   #GRUB_TERMINAL=console

   # The resolution used on graphical terminal
   # note that you can use only modes which your graphic card supports via VBE
   # you can see them in real GRUB with the command `vbeinfo'
   #GRUB_GFXMODE=640x480

   # Uncomment if you don't want GRUB to pass "root=UUID=xxx" parameter to Linux
   #GRUB_DISABLE_LINUX_UUID=true

   # Uncomment to disable generation of recovery mode menu entries
   #GRUB_DISABLE_RECOVERY="true"

   # Uncomment to get a beep at grub start
   #GRUB_INIT_TUNE="480 440 1"

Find the line setting the :code:`GRUB_DEFAULT`, highlighted above. Setting this to 0 means that in the Grub menu, the very first option will be the default.

Set this to the number of the Windows option in your Grub menu (starting to count from 0!) You will have to look this up again, but it is likely that Windows is the third option (i.e. :code:`GRUB_DEFAULT` = 2).

Change the correct line in your configuration file. 

After changing this file, update your configuration:

.. code-block:: sh

   update-grub


Finally, you might notice a wrong time displayed when switching between Windows and Ubuntu.

In order to fix this, in Ubuntu, simply run

.. code-block:: sh

   timedatectl set-local-rtc 1