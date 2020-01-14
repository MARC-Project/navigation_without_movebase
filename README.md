# navigation_without_movebase
A navigation package for MARC project. This package only uses monocular camera and QR code to obtain target pose.   

|Author|Huang Tianjian|
|---|---
|E-mail|117010099@link.cuhk.edu.cn  


To run the package, launch the file `ar_bezier_navigation.launch`.
  
**Attention**: This package depends on `usb_cam`, `ar_track_alvar` and `yocs_velocity_smoother`. You need to install them before compiling this package. In addition, you may also reconfigure the `OpenCV` directory in CMakeLists.txt according to your OpenCV installation environment.  
  
**Hardware support**: Theoretically, as long as your smartcar hardware can receive topics with type `geometry_msgs/Twist` as velocity conmmands, this algorithm can run on the hardware platform. Your smart car should also need a monocular camera and can public image topics.

