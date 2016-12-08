# Localization, Mapping, and Path-planning for a Powered Longboard
## *powerboard_pcl ROS package README*

_Owner: Alan Hong, Northwestern University, MSR 2016_

_Project write-up [available here](https://github.com/hongalan/powerboard/blob/master/writeup.md)._


~~~
Table of Contents
1. Package Description
2. ROS Package Dependencies
3. Launch Files
4. Notes on Package Usage
~~~

### 1. Package Description

The `powerboard_pcl` package takes orientation and speed measurements sent from an Arduino and scans from a Hokuyo scanning laser range finder to perform localization, mapping and navigation.

As of December 2016, the package is able to produce a map of its environment and localize the user's longboard in the map. Development is now moving into navigation through a pre-generated map and obstacle avoidance while maintaining shared autonomy.

### 2. ROS Package Dependencies

`powerboard_pcl` is dependent on the following packages. Upon cloning this package, you may install them by running `rosdep install powerboard_pcl`
    roscpp
    sensor_msgs
    rosserial_python
    gmapping
    map_server
    laser_scan_matcher
    tf

### 3. Launch Files

#### **power_map.launch**
##### Arguments:
- `rviz` default="true"
  - If set true, RViz will be launched and configured to view the `map`, `odom`, and `base_link` frames produced by odometry and laser localization scripts and the map generated by `gmapping`.
- `bag` default="false"
  - If set true, the launch file will use bagged data to produce the map instead of using measurements sent by the sensors.
- `bag_path`
  - If `bag` argument is set true, use this argument to input the path of the bag file
- `laser_port` default = "/dev/hokuyo"
  - Set to the port connected to the Hokuyo scanning laser range finder
  - Default value points to symbolic link produced by [udev rules](https://github.com/hongalan/powerboard/blob/master/powerboard_pcl/resources/99-usb-serial.rules)
- `laser_port` default = "/dev/arduino"
  - Set to the port connected to the Arduino
  - Default value points to symbolic link produced by [udev rules](https://github.com/hongalan/powerboard/blob/master/powerboard_pcl/resources/99-usb-serial.rules)

### 4. Notes on Package Usage

- power_map.launch will start multiple nodes under the `mapper` namespace
- To make use of symbolic links to usb devices, copy `powerboard_pcl/resources/99-usb-serial.rules` to `/etc/udev/rules.d/`.