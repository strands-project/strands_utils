### Odometry Mileometre

This very simple node subscribes to `/odom` and calculates the travelled distance since start in metres.
The result is publish on `/odom_mileage`.

Start with `rosrun odomoetry_mileage odometry_mileage`

If the parametere mileage is present on the rosparam server (e.g. via the datacentre), then the node will use the stored mileage to intialise itself.
While the node is running the mileage parameter is constantly set to the current value and saved to the datacentre. The default save interval
is every 500 odometry messages, which is every ~10 seconds. This can be change via the `save_interval` parametre.

To initialise your node with the current mileage on the robot do: *(Only do this before you launche the node for the first time!)*
* `rostopic echo -n 1 /mileage` and copy that value
* `rosparam set mileage <your mileage value>`
* `rosservice call /config_manager/save_param mileage`
