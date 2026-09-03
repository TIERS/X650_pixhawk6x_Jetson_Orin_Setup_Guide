source ./devel/setup.bash
roslaunch mavros px4.launch & sleep 10;
roslaunch lidar_to_mavros lidar_to_mavros.launch & sleep 5;
rosrun mavros mavcmd long 511 32 8000 0 0 0 0 0;
wait;
