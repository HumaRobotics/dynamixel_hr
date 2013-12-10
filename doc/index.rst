.. dynamixel_hr documentation master file, created by
   sphinx-quickstart on Tue Dec 10 09:08:51 2013.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to dynamixel_hr's documentation!
========================================

dynamixel_hr is a Python 2.7 library for programming Robotis Dynamixel motors directly from python or through the ROS bindings provided.
It also comes with a GUI that allows to quickly identify/configure/manipulate your motors and expose them to a ROS master.

.. toctree::
   :maxdepth: 2






MODELS SUPPORTED
================
At the moment it can handle the following models: AX12*, AX18*, MX28*, MX64*.
However adding your own models is very easy, take a look at the file dxlmotors.py.



INSTALLATION
============

Win 7
-----
Setup drivers for USB2Dynamixel
    - Install FTDI driver for USB2Dynamixel http://www.ftdichip.com/Drivers/CDM/CDM20830_Setup.exe (doc: http://www.ftdichip.com/Drivers/VCP.htm  )
    - Follow instructions from http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/usb2dxl_windows.htm
    - Set USB: Port 21, max baudrate, delay 1

Setup python and pyserial
    - Install Python 2.7: http://www.python.org/ftp/python/2.7.6/python-2.7.6.msi
    - Install pyserial: https://pypi.python.org/packages/any/p/pyserial/pyserial-2.7.win32.exe#md5=21555387937eeb79126cde25abee4b35
    
Linux
-----
Setup python and pyserial
    - Install Python 2.7: sudo apt-get install python2.7
    - Install pyserial: sudo apt-get install python-serial

Access to the serial device (/tty/USB0 by default) needs special rights, so you'll need either to sudo or add your user to the dialout group:
sudo usermod -a -G dialout username
    

ROS Bindings
============

Install ROS Groovy (on Ubuntu 12.04):
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install ros-groovy-desktop-full

Run a ROS master:
  source /opt/ros/groovy/setup.bash
  roscore
  
Run the ToolDynamixelLab with ROS bindings:
  source /opt/ros/groovy/setup.bash
  python ToolDynamixelLab.py --ros

Scan or connect to your motor chain, click on the ROS button to activate bindings (either Raw or international system units SI)


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. automodule:: dxl.dxlchain
   :members:
.. automodule:: dxl.dxlcore
   :members:
.. automodule:: dxl.dxlmotors
   :members:
.. automodule:: dxl.dxlregisters
   :members:
.. automodule:: dxl.dxlros
   :members:
   