irl_audio
=========

A set of ROS packages for ManyEars, IntRoLab's sound localization and
tracking library.

Based on the old manyears_ros distribution, but with various fixes and
catkin compatible.

manyears_ros
------------

This package is a ROS wrapper for the ManyEars sound localization library.

### Kinect usage
ManyEars is compatible and already configured to work with a first generation
Microsoft Kinect.
Since it uses the rt_audio_ros sub-package, your Kinect has to be configured to
be used as a standard USB Audio Class device.
To do so, refer to the HARK-KINECT project:

http://www.hark.jp/wiki.cgi?page=HARK-KINECT+Installation+Instructions+%28as+a+USB+recording+device%29

Then, refer to the 'manyears_node_kinect.launch' launch file in the manyears_ros
package, which will automatically launch both rt_audio_ros for sound cature and
manyears_node for sound localization.
Tracked sources details will appear on the '/tracked_sources' topic, while the
'/source_pose' geometry_msgs/PoseStamped topic will have the location of the
source with the highest energy.
Keep in mind that only the orientation on the XY plane in front of the Kinect is
considered, and so only the angle around Z should be used.

