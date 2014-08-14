This is a calibration tool from the *Hobbit FP7 project* for getting the minimum and maximum disparity
values for the Kinect's v-disparity-based floor detection
and removal and subsequent obstacle detection.

* Adjust the kinect such that the floor in front of the robot is visible.

* `rosrun driveable_floor_calib driveable_floor_calib _disparity_img_topic:=/camera/depth/disparity`

* To show disparity and v-disparitiy, press the button *Connect* in the Qt window.

* The values with the robot standing still (not tilted) will be the nominal
values "NomK" and "NomD" later used in a config file. Press *Nominal* at this position.

* Tilt the robot as much as it should maximally tilt forward
during normal operation (e.g. when breaking). Press *Maximum* at this position.

* Tilt the robot backwards as much as it should maximally
tilt backwards during normal operation (e.g. when breaking during
going over a small step). Press *Minimum* at this position.

* *Save* and close the Qt window. This creates a file `Params.txt` in the current directory. 

* For increased reliability, decrease the value of *MinD* by 1.0 and increase the value of *MaxD* by 1.0 in `Params.txt`.


* If it was not already done, the coordinate transform (tf) from the odometry center of the mobile
platform (/base_link) to the center of the top depth camera’s optical center when it is looking
straight down (/obstacle_link) has to be determined. In the constructor of the class QNode (`qnode.cpp`), the x, y and z
coordinates of obstacle_trans need to be updated accordingly (no rotation between the platform
and virtual laser coordinate system).
