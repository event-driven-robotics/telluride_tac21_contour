Contour following with iCub
======================

yarpserver --write

yarpdev --device transformServer --ROS::enable_ros_publisher false --ROS::enable_ros_subscriber false

gazebo $ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX/share/gazebo/worlds/vte_scenario.sdf

yarprobotinterface --context gazeboCartesianControl --config no_legs.xml

iKinCartesianSolver --context gazeboCartesianControl --part right_arm

./contour_following

Contour following is a task dealing no challange to us as human; we do it instincivly with no effort. But in robotics it requieres a lot of effort. On the one hand the input has to be processed in a meaning full way, on the other hand motor control has to be executed in dependes to the input. These close loop between sensory read out and action control is still troublesom especially when it relies on tactil only.
To tackle this we have designed a contour follwoing task in differnet stages of complexity leading from continues 2d structures to challanging 3d structures. The humanoid robot [iCub](https://www.iit.it/web/icub) implemented in the simulation environment [gazebo](http://gazebosim.org/) and equiped with tactile sensors in the fingertip and palm serves as our object of research. The simlulation environment can be easily accessed using [docker](https://www.docker.com/). For that we have prepaired a ready-to-use [docker image](TODO fill with link).

2d objects | 3d objects
:---------:|:---------:
![2d_objects](https://github.com/event-driven-robotics/telluride_tac21_contour/blob/master/assets/2d_objects.png)  |  ![3d_objects](https://github.com/event-driven-robotics/telluride_tac21_contour/blob/master/assets/3d_objects.png)


 
