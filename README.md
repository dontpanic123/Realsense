# Realsense



This folder contains a number of codes related to the reconstruction of the blind spot of the forklift

***Keywords: RealSense D435, camera calibration, environment reconstruction, Gstreamer***



Code function introduction:

***example.h / example_server.hpp / example_usb.hpp***   ------    The header file mainly contains some functions for point cloud rendering in OpenGL. The purpose of having multiple similar header files is that when the parameters of one header file are modified, it will not affect the operation of other programs.

***Automatic_camera.cpp***      ------     All D435 cameras connected via USB automatically save a picture every five seconds

***opencv_calibration.cpp***   ------     Use random patterns for opencv multi-camera calibration, added timing function.

***Gst_PointCloud.cpp***        ------     Use Gstreamer to receive video from different ports in Windowsx system, convert it to point cloud and display, Only display data from one camera.

***gpu_multi_test.cpp***     ------     Simultaneous display of multiple point clouds on the GPU

***gst_multi.cpp***       ------      Simultaneously read and display multiple camera data transmitted via Gstreamer

The process of the gst_multi is shown in the figure below
![image](https://github.com/dontpanic123/Realsense/blob/master/gst_pointcloud.png?raw=true)

***rs_server_opengl.cpp***      ------      Simultaneously read and display multiple camera data transmitted through rs-server

***usb_multi.cpp***   ------      Read and display multiple camera data transmitted via usb at the same time

The process of the usb_multi is shown in the figure below
![image](https://github.com/dontpanic123/Realsense/blob/master/usb_multi.png?raw=true)

***usb_postprocess.cpp***      ------     Simultaneously read and display multiple camera data transmitted via usb, added multiple point cloud post-processing functions



