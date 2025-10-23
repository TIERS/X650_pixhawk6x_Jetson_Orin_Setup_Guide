roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540  & sleep 15;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 10;
roslaunch fast_lio mapping_mid360.launch & sleep 10;
rosrun mavros mavcmd long 511 32 8000 0 0 0 0 0;
wait;
