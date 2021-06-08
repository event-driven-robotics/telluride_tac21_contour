Contour following with iCub
======================

yarpserver --write

yarpdev --device transformServer --ROS::enable_ros_publisher false --ROS::enable_ros_subscriber false

gazebo $ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX/share/gazebo/worlds/vte_scenario.sdf

yarprobotinterface --context gazeboCartesianControl --config no_legs.xml

iKinCartesianSolver --context gazeboCartesianControl --part right_arm

./contour_following

Contour following is a task dealing no challenge to us as human; we do it instinctively with no effort. But in robotics, requires a lot of effort. On the one hand, the input has to be processed in a meaningful way, on the other hand, motor control has to be executed in depends on the input. This close loop between sensory read-out and action control is still troublesome especially when it relies on tactile only.
To tackle this we have designed a contour following task in different stages of complexity leading from continuous 2d structures to challenging 3d structures. The humanoid robot [iCub](https://www.iit.it/web/icub) implemented in the simulation environment [gazebo](http://gazebosim.org/) and equipped with [tactile sensors](http://wiki.icub.org/wiki/Tactile_sensors_(aka_Skin)) in the fingertip and palm serves as our object of research. (TODO fill more info about sensors and placment) The simulation environment can be easily accessed using [docker](https://www.docker.com/). For that, we have prepared a ready-to-use [docker image](TODO fill with link).

2d objects | 3d objects
:---------:|:---------:
![2d_objects](https://github.com/event-driven-robotics/telluride_tac21_contour/blob/master/assets/2d_objects.png)  |  ![3d_objects](https://github.com/event-driven-robotics/telluride_tac21_contour/blob/master/assets/3d_objects.png) 

### Task

The robot will always start at the top right corner of the objects with no contact with the contours. Your task is to give iCub the ability to follow the different contours, first in 2d. Following the circle for instance will be the easiest task as it is continuous and has no abrupt changes. The triangle otherwise includes very sharp edges and the robot needs to adapt its movement fast and correctly to keep track. At least we have included an object with crossing contours to test your solutions on sustainability.

For those who master the first challange we have inlcuded the 3d objects. Now it is not sufficient to rely on the spacial component (e.g. which sensor measures the force), but the amount of force gets an important feature to keep contact. Here again the difficulty is wide spread from continues, wavelike structures to some with sharp edges and crossing contours. The 3d objects are similar to the 2d with just adding depth dependencies on top. 

If you are able to solve all of the given tasks we are still able to provide you even more challanging ones. The content of that can be discussed in the regular meetings.

Good luck and have fun
