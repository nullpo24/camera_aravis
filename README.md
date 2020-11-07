camera_aravis for ROS2

This is a [ROS2](http://ros.org) package for the [Aravis GigEVision
library](http://live.gnome.org/Aravis). It is open source, under the
LGPL (like Aravis itself).

## Prerequisite
Install Aravis (open source project), one of the drivers for GigE Vision.
```
$ sudo apt install aravis-tools
```

## Build
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Run
```
$ ./camenode [VendorName]-[Serial]
```

------------------------
The basic command to run camera_aravis:

    $ ros2 run camera_aravis camnode

------------------------
This ROS node publishes messages image_raw and camera_info for a specified camera.  It supports 
a variety of camera features via the ROS reconfigure_gui, including the following:
* ExposureAuto         (string: Off, Once, Continuous)
* GainAuto             (string: Off, Once, Continuous)
* ExposureTimeAbs      (float)
* Gain                 (float)
* AcquisitionMode      (string: Continuous, SingleFrame, MultiFrame)
* AcquisitionFrameRate (float)
* TriggerMode          (string: Off, On)
* TriggerSource        (string: Software, Line1, Line2)
* softwaretriggerrate  (float)
* frame_id             (string)
* FocusPos             (integer)
* mtu                  (integer)

Note that the above are also the ROS parameter names of their respective feature.  You may
set initial values for the camera by setting ROS parameters in the camera's namespace.

In addition to the above features, this driver now supports (almost) every feature of every camera,
you just have to know how the feature is specified; each GenICam-based camera contains 
an XML file onboard, and by viewing this file you can determine which ROS parameters to set 
for camera_aravis to write to the camera.

You can use arv-tool-0.6 to see the feature list 
and the XML file (e.g. "arv-tool-0.6 --name=Basler-21285878 features")
