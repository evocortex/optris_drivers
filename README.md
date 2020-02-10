# optris_drivers with ROS support for focus motor adjustments

**ROS drivers for Optris thermal imagers**

Forked from: [evocortex/optris_drivers](https://github.com/evocortex/optris_drivers)

A fork of the optris-drivers ROS package which makes using optris cameras in ROS possible. This fork adds features like manual focus control via ROS topics.


## Running this node:

See the instructions in the original repo.


### Subscribed topics:

`/optris_focus_position` (`std_msgs/Float32`):
The `/optris_imager_node` subscribes to this topic and sets the focus accordingly. The position of the focus motor can be adjusted from 0% (for objects near the camera) to 100% by sending a value between `0.0` and `100.0`.

One possible implementation of an autofocus for Optris cameras is a laserscanner that measures the distance in the cameras viewing direction and a corresponding node that has a lookup table with several distances and according focus positions which are then published to the `/optris_focus_position` topic.

