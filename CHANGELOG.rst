^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pololu_smc_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2014-02-12)
------------------
* Add VIN, rename Channel to more general Input
  Also fixed a possible segfault on disconnect in the input timer routines
* Add udev rules
* Add device list tool
* Add lookup for SMC model name
* Contributors: Scott K Logan

0.1.6 (2014-02-10)
------------------
* Actual topics for channel data (up to 1000Hz polling)
* Remove channel variables from diagnostics
* Better defaults for config
* Fix an enum in channel DynRe
* Contributors: Scott K Logan

0.1.5 (2014-02-09)
------------------
* Add rosdoc.yaml
* Fix doxygen in DynRe config files
* Switched to using JointTrajectory instead of Float32 for speed
  Topic also changed to joint_trajectory within the public namespace
* Speed is now -3200 to 3200, not -1 to 1
* Added final missing vars to diagnostics
* Added dynamic_reconfigure for channels and limits
* Contributors: Scott K Logan

0.1.4 (2014-02-08)
------------------
* Fixed driver nodelet
* Make settings respect param server, lock in serial
  Settings behavior has changed to:
  1) initially query device for settings
  2) make settings match those explicitly set in param server
  3) update param server with newly merged settings
  After initial startup, parameter server should have all settings in it. This makes subsequent disconnect/reconnect cycles have the exact same settings that the device had before the disconnect/reconnect cycle, even if device settings changed or were lost.
  Also, once the driver is connected to a device, it will only connect to that exact device on a subsequent disconnect/reconnect cycle. If you need to swap devices, you need to restart the driver. If the parameter server is not restarted as well and the node name doesn't change, then the old settings will be sent to the new device.
* Initial failure clarification
* Fix dependency on gencfg
* Contributors: Scott K Logan

0.1.3 (2013-02-23)
------------------
* Removed extra export stuff in package.xml
* Added driver_base depends
* Contributors: Scott K Logan

0.1.2 (2013-02-10 21:46)
------------------------
* Update to version 0.1.2
* Tagging as version 0.1.1
* Contributors: mattrichard

0.1.1 (2013-02-10 21:17)
------------------------
* Fixed libusb dependency name
* Contributors: Walter

0.1.0 (2013-01-24)
------------------
* Fixed settings update
* Fixed update frequency diagnostic
  Forgot to tick( ) it!
* Re-worked library linking pipeline
  This should make things compile on Ubuntu as well. I don't understand this fully, but it seems that Ubuntu has trouble linking C libraries later, and Fedora doesn't...
* Fixed catkin_package arguments
  This should only affect dependant packages
* Added libsmc_driver.so to catkin LIBRARIES
  Should have only affected dependant packages
* Contributors: Scott K Logan
