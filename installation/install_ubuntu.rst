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

#. Download `Ubuntu 20.04 Desktop <https://releases.ubuntu.com/20.04/>`_ from the offical Website.

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



