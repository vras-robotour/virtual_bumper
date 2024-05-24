# virtual_bumper
ROS tools to prevent robot from hitting obstacles.

- [Overview](#overview)
- [How to use](#how-to-use)
- [License](#license)

## Overview
This is a ROS package that provides a virtual bumper for a robot. The virtual bumper is a safety feature that prevents the robot from hitting obstacles by stopping the robot if it is close to an obstacle. 

We provide a node that takes in a point cloud and a velocity command and outputs a velocity command modified to avoid any obstacles. 

The package was developed and tested on Ubuntu 20.04 with ROS Noetic and should work with other versions of ROS as well.

## How to use

The script can be run either with the `rosrun` command or as an executable. However we would recommend launching it with a provided launch file. The launch file will set the parameters for the script and launch the node. With the launch file you can also change the topic of the incoming point cloud, input command velocity as well as the output command velocity. The launch file is located in the `./launch/` directory. The launch file has two arguments:
- `robot_name` - the name of the robot, we consider four different robots: `ctu-robot`, `flip`, `marv-robot`, and `spot`, the script will use the default parameters and correct topic names for the selected robot
- `debug` - if true, the script will publish the obstacle point cloud with the centroids of the obstacles, default is false

To launch the script with the launch file, run the following command:
```bash
roslaunch virtual_bumper virtual_bumper.launch robot:=<robot_name> debug:=true
```

The script takes several parameters from the ROS parameter server:
- `~robot_width` - the width of the robot, default is 0.6
- `~robot_length` - the length of the robot, default is 0.8
- `~robot_frame` - the frame of the robot, default is `base_link`
- `~clearance_box` - the (x,y,z) size of the clearance box around the robot with a minimum and maximum value for each axis, default is (-1.0, 1.0), (-0.75, 0.75), (0.05, 0.8)
- `~cloud_max_delay` - the maximum delay in seconds for a point cloud to be considered valid, default is 0.6
- `~min_points_obstacle` - the minimum number of points in a point cloud to be considered an obstacle, default is 20
- `~debug`- if true, the script will publish the obstacle point cloud with the centroids of the obstacles, default is false

## License
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/vras-robotour/virtual_bumper/blob/master/LICENSE)

