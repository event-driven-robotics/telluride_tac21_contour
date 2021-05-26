Contour following with iCub
======================

yarpserver --write

yarpdev --device transformServer --ROS::enable_ros_publisher false --ROS::enable_ros_subscriber false

gazebo $ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX/share/gazebo/worlds/vte_scenario.sdf

yarprobotinterface --context gazeboCartesianControl --config no_legs.xml

iKinCartesianSolver --context gazeboCartesianControl --part right_arm

./contour_following