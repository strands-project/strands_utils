nav_goals_generator
===================

ROS Service to generate *n* 2D navigation goals in a given map. The service
also takes an *inflation radius* which ressembles the robot's footprint into
account.


# Usage

First make sure that an occupancy grid map (nav_msgs/OccupancyGrid) is
published on a topic, e.g. `/map`. Second, launch the service as follows:

```
roslaunch nav_goal_generation nav_goal_generation.launch
```

You can send now a service request:

```
rosservice call nav_goals_generator 100 0.5
```

whereby the first argument is the number of goal loactions to be generated
(here 100) and and the second argument is the inflation radius of the robot's
footprint.  The result of the pose generation is additionally published on the
topic `/nav_goals` in order to visualize the result in RVIZ.

# Known Issues

The service fails if there is no map topic available or no message has been
published on this topic after the service has been started. As a workaround,
the map server could be started after this service.
 





