# navigation_without_movebase
A navigation package for MARC project. This package only uses monocular camera and QR code to obtain target pose.  
To run the package, launch the file _ar_bezier_navigation.launch_.

**Attention**: This package depends on _usb_cam_, _ar_track_alvar_ and _yocs_velocity_smoother_. You need to install them before compiling this package. In addition, you may also reconfigure the OpenCV directory in CMakeLists.txt according to your OpenCV installation environment.
