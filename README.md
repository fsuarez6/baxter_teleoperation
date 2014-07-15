baxter-teleop 
=============

ROS packages developed by **TODO**. On going development continues in the hydro-devel branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/baxter_teleop.svg?branch=hydro-devel)](https://travis-ci.org/fsuarez6/baxter_teleop)


## Installation

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool merge https://raw.github.com/fsuarez6/moveit_kinematics_interface/hydro-devel/moveit_kinematics_interface.rosinstall
wstool merge https://raw.github.com/fsuarez6/baxter_teleop/hydro-devel/baxter_teleop.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the package:
```
roslaunch baxter_teleop ?.launch
``` 

## Changelog
TODO

## Roadmap

### 0.1.0 (2014-09-30)
* Initial release

## Tutorials
TODO

