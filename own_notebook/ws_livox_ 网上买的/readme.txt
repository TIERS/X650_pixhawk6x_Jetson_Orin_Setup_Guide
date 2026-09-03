提前下载好Livox SDK2配好雷达的静态IP后，然后进入ws_livox/src/livox_ros_driver2里面打开终端输入sudo chmod 777 *.sh然后
输入./build.sh ROS1之后等待编译完成，然后去到driver2里面的config文件夹点击mid360的那个修改你的雷达IP还有ID，然后在ws
_livox/shfiles文件夹下sudo chmod 777 *.sh，然后./lidar_to_mavros.sh即可运行定位