Install Ubuntu
##############

.. attention:: Since we are going to install a new/second operating system on your device, that might be a good opportunity to create a backup just in case anything goes really wrong! We are not responsible for any damage or loss of data that might result from any of the following instructions.

.. note:: For those of you who have their windows encrypted with bitlocker you might have to enter your bitlocker key the next time you boot windows after you have installed Ubuntu. You should save your bitlocker key (you can find it `here <https://account.microsoft.com/devices/>`_). Alternatively you can turn off bitlocker encryption.

#. Resize your Windows partition. Hit :kbd:`Win` + :kbd:`R` and enter :code:`diskmgmt.msg` and click **OK**

   .. image:: /res/images/windows_execute.png

#. Choose the partition on which you want to install Ubuntu (in most cases this will be :file:`(C:)`.

#. Resize the partition to create unallocated space where we can install Ubuntu. 30GB of free space might be enough for Ubuntu, but we recommend at least 60GB if you can afford it.

   .. image:: /res/images/windows_disk_manager.png

#. The result could look similiar to this:

   .. image:: /res/images/windows_disk_manager_result.png

#. Download `Ubuntu 18.04 Desktop <https://releases.ubuntu.com/18.04/>`_ from the offical Website.

#. Download `Etcher <https://www.balena.io/etcher/>`_. 

#. Insert an USB stick with at least 4GB of memory.

#. Start Etcher and select the downloaded Ubuntu image and the inserted USB device as target.

   .. image:: /res/images/etcher.png
 
#. Make sure you **really** selected the correct target. Otherwise all your data on the wrongly selected target might get deleted.

#. Click **Flash!**.

#. Reboot your system after flashing is done and enter your BIOS/UEFI. During Bootup. To do so you have to hit a certain key, depending on your hardware. For Dell commputers you probably need to hit :kbd:`F12` during the DELL splashscreen. For Lenovo it is :kbd:`Enter`. This opens a dialog where you can choose to enter your BIOS settings.

#. Find the settings to disable Fast Startup (if available) and Secure Boot.

#. Save your changes, reboot and select the medium you want to boot from during the splashscreen (again :kbd:`F12` for Dell or :kbd:`Enter` for Lenovo). Now you want to boot from the Ubuntu USB drive.

#. Choose :code:`Try Ubuntu without installing`. This will boot Ubuntu from the USB stick.

#. On the desktop there will be an icon for installing Ubuntu. Double click it to launch the installation wizard.

#. Choose the option to install additional drivers. It is also a good idea to connect to a nearby WiFi or Ethernet.

   .. image:: /res/images/ubuntu_additional_drivers.png

#. In the last step of the installation the wizard asks you if you want to install Ubuntu alongside windows because it detects your windows installation and the free disk space we created before. Chose this option and click **Install Now**.

   .. image:: /res/images/ubuntu_alongside.png



