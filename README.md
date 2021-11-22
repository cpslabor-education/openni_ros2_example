# openni_ros2_example
A simple ROS2-based example for ASUS Xtion camera with OpenNI. This package is mostly for educational purpose, use at your own risk.

## Prerequisites
Naturally, you shall need ROS2. You can build by yourself or use publicly available packages. Here, the simplest instructions are provided:
- For Linux, follow instructions as listed here: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- For Windows, follow instructions as listed here: https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html

Ensure that the __colcon__ toolchain is installed. You can issue the following command to install the toolchain on your system:
```
pip install -U colcon-common-extensions
```
Or if you have a system with both Python 2.x and Python 3.x interpreters:
```
pip3 install -U colcon-common-extensions
```

You shall need OpenNI drivers. 
- Under Linux, installing OpenNI drivers are relatively simple.
- On Windows, you can use the OpenNI SDK, available here (Windows 10): https://structure.io/openni, more precisely: https://s3.amazonaws.com/com.occipital.openni/OpenNI-Windows-x64-2.2.0.33.zip


## Usage
Install this package by cloning this repository (e.g. to your _dev_ws/src_ folder):
```bash
git clone https://github.com/robotlabor-education/openni_ros2_example.git
```
Then, you can invoke colcon build in the workspace folder (e.g. _dev_ws_):
```
colcon build
```
On Linux, you can create symbolic links for your build to spare storage space:
```
colcon build --symlink-install
```

### Visual Studio 2019 specifics
Under Windows colcon generates the proper Visual Studio 2019 files (solution and vcxproj files). These are generated in your workspace and corresponding package folders (e.g. _dev_ws/build/openni_camera_). You can start and build your application as a conventional Visual Studio project.

### Protip for Visual Studio 2019 based builds
The messages are hidden during build. Therefore, the developer can't get real insight on what error happened during build. You can enable this during build:
```
colcon build --event-handlers console_direct+
```

## Video instructions in Hungarian
Kövesd a linket a videóhoz az insturkciókhoz: https://youtu.be/InL-u0dnvCA