depthimage_to_laserscan
=======================

Converts a depth image to a laser scan for use with navigation and localization.

ROS Wiki Page:
http://www.ros.org/wiki/depthimage_to_laserscan

The package is renamed to allow it to exist side-by-side with the original depthimage_to_laserscan package without any conflicts. A sample launch file for the turtlebot is included.

*Note:*
The revision is added for noetic version of ROS that from the forked repo.

### Parameters and Getting it Working

You will need to change the `depth_image` arg to match yours (the camera info topic is determined automatically based on the depth image's topic). The `scan` arg specifies the topic that the generated laserscan will be published on. The nodelet expects a rectified depth image, though the raw image can work as long as the camera's distortion is minimal; YMMV. Some pdeth image cllision avoidance or navigation strategies use decimated depth maps for faster processing. Here, it doesn't make much difference since computation time scales sublinearly with the number of pixels.

`scan_height`: param determines how much of the images is used when generating the laserscan; it can be set  to anything from 1 to (image_height-1). There's no compelling reason to deviate from using the largest value. <BR>
`output_frame_id`: set to match the frame_id of your depth camera- not the camera's optical frame_id! <BR>
`range_min`: ignore anything closer than this value; set it to the nearest effective range of your camera. <BR>
`floor_dist`: set it to the vertical distance of the depth camera above the floor, this allows the floor to be ignored when generating the laserscan. 
Some trial and error will be needed to find the best value for your use, as the filtering assumes that the camera stays perfectly level with the groundplane, meaning that if the robot pitches forward (such as when slowing rapidly) part of the floor may be falsely registered as an obstacle. A few cm extra is usually enough. <BR>
`overhead_dist`: the vertical distance from the camera to the highest point on the robot. It serves a similar purpose to `floor_dist`, except that it filters out obstacles that are too high to collide with the robot.

Note that all of the parameters can be dynamically reconfigured, so it shouldn't take too long to find good values for them.
Just like the original implementation, the nodelet only performs the computations if something subscribes to it, so you can leave it running all the time without negligible cost.

The nodelet publishes the `mask` used to filter points on the topic `mask_image`.  You can visualize this as a pointcloud using [point cloud visualization](http://wiki.ros.org/depth_image_proc#depth_image_proc.2Fpoint_cloud_xyz) by remapping `camera_info` to your depth camera's camera info topic and remapping `image_rect` to `mask_image` (or whatever you choose to remap it to). It visualizes the upper and lower bounds in rviz relative to the robot. As a nodelet, it has negligible cost when nothing subscribes to the generated pointcloud.
