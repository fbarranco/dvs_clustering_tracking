# DVS Segmentation 

This package implements a simple segmentation algorithm using DVS data (event based camera). 

## Getting Started

Clone or donwload the project. After that, first source your *ROS distro* files:
```
$ source /opt/ros/<your_distro>/setup.bash
```

And follow the next files to compile the package:
```
$ mkdir -p ~/dvs_segmentation/src
$ cd ~/dvs_segmentation/
$ catkin_make
```

After this, you will have a copy of the project for testing.

### Prerequisites

In order to install the project, you will need a ROS distribution running in your platform. This project was developed and tested using *kinetic*. See more about the installation and tutorial examples in the [ROS website](http://wiki.ros.org/ROS/Tutorials/).

### Installing

In order to install all the dependencies you will have to install:
* [`rpg_dvs_ros`](https://github.com/uzh-rpg/rpg_dvs_ros): where we will use `dvs_ros_driver`(and then `dvs_driver`), `dvs_msgs`, `dvs_renderer`, `libcaer_catkin` (and then `libcaer`)
* [`catkin_simple`](https://github.com/catkin/catkin_simple)

## Running the tests

After the compiling the project code, source the *setup.bash* file

```
:~/dvs_segmentation$ source devel/setup.bash
```
Next, we will show two examples for running some of the tests:

### Running the dvs_renderer
If you do not have the DVS camera and want to run a file, simply run:
```
:~/dvs_segmentation$ rosbag play ~/bagfiles/shapes_rotation.bag -l
```
where in `~/bagfiles` we have the rosbag file with dvs events ([rosbag files dataset] (http://rpg.ifi.uzh.ch/davis_data.html)) 

Or, if you have your own camera, just run in a different console (you'll need to source again the setup.bash files in this new console):
```
$ roslaunch dvs_renderer dvs_mono.launch
```

### Running dvs_meanshift
In case there is no attached DVS, just run the same rosbag example:
```
:~/dvs_segmentation$ rosbag play ~/bagfiles/shapes_rotation.bag -l
```
where in `~/bagfiles` we have the rosbag file with dvs events ([rosbag files dataset] (http://rpg.ifi.uzh.ch/davis_data.html))

Then, run in a different console (you'll need to source again the *setup.bash* files in this new console):
```
$ roslaunch dvs_meanshift dvs_segmentation.launch
```

## Authors

* **Francisco Barranco** - *University of Granada*
Please report problems, bugs, or suggestions to fbarranco_at_ugr_dot_es (Replace _at_ by @ and _dot_ by .).

## Acknowledgments

* Project X

## License

Copyright (C) 2015 Francisco Barranco, 11/18/2015, University of Granada.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

