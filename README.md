ROS 2 - OpenCV interoperability show case package
===================================================

`ROS 2` demo package to illustrate interoperability with `OpenCV`.</br>
It was tested with ROS 2 Humble but should also work with other distribution since no tweaks were added to make sure anyone can use it.</br>
This package provides two nodes:
- `CameraMockerNode`: generating image from OpenCV and publishing it through `image_provider` topic
- `DataProcessorNode`: susbscribing to `image_provider` topic and processing received image with OpenCV then publishing processed image through `processed_image` topic

What to expect
===================================================
This package provides a launch file to automatically launch the two nodes described above, one publishing an image to `image_provider` and a second subscribing to this topic and as callback perform basic image processing then publish the result in the `processed_image` topic.</br>
The animation below illustrate the expected behavior
![](ros2-opencv-interoperability.gif)

How to use
===================================================
Go into your ROS2 workspace
```
cd ~/ros2_ws/src
```
Clone the repository
```
git clone https://github.com/BBO-repo/ros2-opencv.git
```
Return to your workspace folder and rebuild your workspace
```
cd ~/ros2_ws
colcon build --symlink-install
```
Source your `install/setup.bash`
```
source install/setup.bash
```
Launch the package launch file
```
ros2 launch src/ros2-opencv/launch/ros2-opencv.launch.xml rviz_config:=src/ros2-opencv/config/ros2_opencv.rviz
```
