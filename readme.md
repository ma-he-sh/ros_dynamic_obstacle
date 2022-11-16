## Running ROS

```shell
cd catkin_ws
source devel/setup.sh

roslaunch <package name> <launch file>
roslaunch dynamic_obstacle realsense_cam.launch
```

```markdown
http://wiki.ros.org/frontier_exploration
https://github.com/mit-acl/cadrl_ros
http://wiki.ros.org/teb_local_planner/Tutorials/Incorporate%20dynamic%20obstacles
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
https://github.com/rst-tu-dortmund/costmap_converter
https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/
http://wiki.ros.org/costmap_2d
http://wiki.ros.org/tf#static_transform_publisher
https://www.youtube.com/watch?v=0C0gOsLoP9k
http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://library.isr.ist.utl.pt/docs/roswiki/pcl_ros.html

https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/scripts/marker_generation.py
```


### Setup
```shell
sudo apt install ros-melodic-ros-numpy
sudo apt install ros-noetic-pcl-ros
pip install open3d_ros_helper
```