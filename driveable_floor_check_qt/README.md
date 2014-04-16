This is a tool from the Hobbit FP7 project for floor detection and removal and subsequent obstacle detection.

It requires calibration parameters stored in the file `Params.txt`as described in *https://github.com/strands-project/strands_utils/tree/hydro-devel/driveable_floor_calib*.

This package is Qt based. The console version can be found in *https://github.com/strands-project/strands_utils/tree/hydro-devel/driveable_floor_check_console*.

To start floor detection, do:

```rosrun driveable_floor_check_qt driveable_floor_check_qt Params.txt _cam_info_topic:=/camera/depth_registered/camera_info _disparity_img_topic:=/camera/depth_registered/disparity _active_topic:=/camera/active```

and press *Connect* in the Qt window.

To show the processing results, tick the checkbox *Show*. In the second last image, it shows the occupancy grid with the virtual 2D laser scan. Blue indicates the undisturbed laser beam, gray to white means occupied cells of the grid, and red parts are parts of the grid that the laser beams did not reach or cover.

A part of the robot is visible in the bottom of the camera images. To avoid misinterpretation as obstacle, this part of the occupancy grid needs to be masked out. Given there is free space in front of the robot and the mask is okay, the bottom of the occupancy grid should be blue only. If there are close-by permanent or sporadic laser hits, the masked-out area needs to be adjusted within the method *MaskPlatformFromObstacleGrid* of the source file `qnode.cpp`. Uncomment the code under ‘for setting the blanking rectangle’ and adjust the values for *BlankStart*, *BlankWidth* and *BlankHeight*. 

The algorithm subscribes to the corresponding ROS topics and publishes a resulting virtual 2D laser scan `/obstacle_scan`.
