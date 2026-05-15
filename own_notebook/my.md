# X650 Setup

## Drone

- **Wi-Fi**：`TIARIAS`
- **SSH**：`ssh x650@192.168.50.240`  
- **Password**：`future`

router：192.168.7.1
asus：future2025 but minh destroyed it
drone ip: 192.168.7.240
my dell laptop ip: 192.168.7.172
dog ip: 192.168.7.225



dog's own sensors' ip:192.168.1.xx



mocap needs TIAIRS wifi:
mocap ip: 192.168.50.213
my dell laptop ip: 192.168.50.172
drone ip: 192.168.50.240



zerotier:
drone ip: 192.168.194.177
my dell laptop ip: 192.168.194.57
dog ip: 192.168.194.228



这台6x上的px4的默认ip仍然是192.168.0.3
jetson主机ip：192.168.0.1
mid360ip：192.168.0.2
fc ip：192.168.0.3



I/O PWM out= main out
FMU pwm out= aux 连接的马达
telem2 内部已经连接到 jetson,我的实验里面不上

mavlink has 3 instances.
mav_0 is in telem1, connected to skydriod rc
mav_1 is in telem3, connect to the normal telemetry radio
mav_2 is in ethernet, connect to switch on jetsonboard, so it's onboard mode.


所以应该设置成如下：

官网的设置会让jetson的无线网都无法联网，不能用那个设置。

```bash
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.0.1/24

```



# JetPack 5.1.2  ros1 Noetic

### realsense Configuration
px4版本：1.15.4

RC IN:遥控器
i2c连接了range finder

qgc4.4.4版本 https://docs.qgroundcontrol.com/Stable_V4.4/en/



串口：

“UART1” 指的是 Jetson 主板上的第一个通用异步收发传输器（Universal Asynchronous Receiver/Transmitter）接口，也就是串口中的一个硬件控制单元。你可以把它理解为主板上标号为1的串口控制器。


but!!!!!在我板子上实际连接的是/dev/ttyTHS0

await drone.connect(system_address="serial:///dev/ttyTHS0:921600")

若你追求 高速稳定、多设备互联，推荐用 以太网 + UDP 方式连接 Pixhawk。


在 onboard模式下，只能使用Client 模式，主动连接
await drone.connect(system_address="udp://192.168.0.3:14540")

------------------------------------------------------------------------------------
先用一个终端建立连接：之后可以立马关闭  有时又用不上

roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540

git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

-----------------------------------------------------


里程计把 camera_init 和 body 联系起来：原始只有 camera_init  和 body 这个节点

odom：是原始机体向上的的位置
camera_init: 激光雷达的初始位置

body：激光雷达一直动，显示的位置
baselink 是机体一直动的位置


livox ros driver 2 读取雷达发送话题 所以等于rosbag

	<node pkg="tf2_ros" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="-0.05763 0 -0.14743 0 -2.3562 0 body base_link"/><!-- this body is actually lidar's body-->

	<node pkg="tf2_ros" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
        args="0.0635 0 -0.1450 0 2.3562 0 odom camera_init"/>


激光探测测距仪内部集成了 IMU芯片（3轴加速度计和3轴陀螺仪）：默认情况下，上电后即开始以200Hz 频率推送IMU数据（可通过上位机开启或关闭）。数据内容包括3轴加速度以及 轴角速度，方向与点云坐标系相固，在点云坐标系下IMU芯片的位置为（×=11.0mm，
y-23.29 mm, Z= 44.12 mm ) -
具体通信协议和数据格式请查看通信协议相关章节。


rviz_MID360.launch	Connect to MID360 LiDAR device
Publish pointcloud2 format data，Autoload rviz


msg_MID360.launch	Connect to MID360 LiDAR device
Publish livox customized pointcloud data


---------------------------
Minimum Return Altitude
By default the minimum return altitude is set using RTL_RETURN_ALT, and the vehicle will just return at the higher of RTL_RETURN_ALT or the initial vehicle altitude.
The minimum return altitude can be further configured using RTL_CONE_ANG, which together with RTL_RETURN_ALT defines a half cone centered around the destination landing point. The cone angle allows a lower minimum return altitude when the return mode is executed close to the destination. This is useful when there are few obstacles near the destination, because it may reduce the minimum height that the vehicle needs to ascend before landing, and hence power consumption and time to land.
￼
The cone affects the minimum return altitude if return mode is triggered within the cylinder defined by the maximum cone radius and RTL_RETURN_ALT: outside this cyclinder RTL_RETURN_ALT is used. Inside the code the minimum return altitude is the intersection of the vehicle position with the cone, or RTL_DESCEND_ALT (whichever is higher). In other words, the vehicle must always ascend to at least RTL_DESCEND_ALT if below that value.
For more information on this return type see Home/Rally Point Return Type (RTL_TYPE=0)


芬兰使用 低功率下433，或者 868，不能使用915

每次无人机放在原地，所有软件打开后再飞，  稳定 5 分钟
-------------------------------------------
装了 新的payload之后，do sensors calibration and auto tuning


!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

indoor：直接只依赖于 lio 关闭磁力计和 gps
EKF2_EV_CTRL ：horizontal position，vertical position，yaw ;yaw will use ev as reference
EKF2_MAG_TYPE：none 
EKF2_HGT_REF：vision
EKF2_gps_CTRL：first three closed
SYS_HAS_MAG=0
SYS_HAS_gps：disabled

CAL_MAG0_PRIO:Disabled(medium)
CAL_MAG1_PRIO:disabled(high)
CAL_MAG2_PRIO:disabled(hig


outdoor：
EKF2_EV_CTRL： 0
EKF2_MAG_TYPE：automatic
EKF2_HGT_REF：gps
EKF2_gps_CTRL：first three open
SYS_HAS_MAG=1
SYS_HAS_gps：enabled

CAL_MAG0_PRIO:medium(default)
CAL_MAG1_PRIO:high
CAL_MAG2_PRIO:high


————————————
这个几个参数要设置一下：距离 cog 的距离  我是把gps那个底座的平面当做了重心!!!!!！！！！！！！！！！！！！！！
EKF2_GPS_POS_X, EKF2_GPS_POS_Y and EKF2_GPS_POS_Z.


The common rangefinder configuration is specified using EKF2_RNG_* parameters. These include (non exhaustively):
* EKF2_RNG_POS_X, EKF2_RNG_POS_Y, EKF2_RNG_POS_Z - offset of the rangefinder from the vehicle centre of gravity in X, Y, Z directions.


———————————



Yaw（偏航）：指的是无人机 机头相对于地理坐标系（通常是 NED，North-East-Down）北方向的转角。


————————————————————


1. Accelerometer（加速度计） 内置
作用：测量无人机在 机体系 (body frame) 下的线性加速度 (m/s²)。
坐标系：机体系
X 轴：机头指向前
Y 轴：机翼右侧
Z 轴：垂直向下（右手坐标系，NED frame 标准）。

1. Gyroscope（陀螺仪） 内置
作用：测量无人机在 机体系 下的角速度 (rad/s)。
坐标系：机体系 (roll/pitch/yaw 的变化率绕 X/Y/Z)。

EKF2_IMU_POS_X 

3. Magnetometer (Compass)（磁力计/电子罗盘）
作用：Yaw
坐标系：输出值先在 机体系 下测量，再转换到 地理坐标系 (NED) 里用于姿态估计。



Internal compasses are not recommended for real use as a heading source, because the performance is almost always very poor.
This is particularly true on on small vehicles where the flight controller has to be mounted close to motor/ESC power lines and other sources of electromagnetic interference. While they may be better on larger vehicles (e.g. VTOL), where it is possible to reduce electromagnetic interference by mounting the flight controller a long way from power supply lines, an external compass will almost always be better.


In order to compensate for the relative motion between the receiver and the CoG, you should configure the following parameters to set the offsets: EKF2_GPS_POS_X, EKF2_GPS_POS_Y and EKF2_GPS_POS_Z.
This is important because the body frame estimated by the EKF will converge on the location of the GNSS module and assume it to be at the CoG. If the GNSS module is significantly offset from the CoG, then rotation around the COG will be interpreted as an altitude change, which in some flight modes (such as position mode) will result in unnecessary corrections.


It is particularly important if using RTK GNSS which has centimeter-level accuracy, because if the offsets are not set then GNSS measurements will often be rejected as inconsistent with the current EFK estimate.


While no further configuration should be required, developers who wish to disable/enable compasses for any reason, such as testing, can do so using the compass parameters. These are prefixed with CALMAGx (where x=0-3):
* CAL_MAGn_ROT can be used to determine which compasses are internal. A compass is internal if CAL_MAGn_ROT==1.
* CAL_MAGx_PRIO sets the relative compass priority and can be used to disable a compass.


4. Barometers（气压计）
作用：测量大气压强，推算相对高度。
坐标系：数据本身是标量，但在 PX4 中作为 地理坐标系 NED 的 Z 轴（高度/深度）信息输入。

* Enable/Disable barometers as data source for Height estimation using the EKF2_BARO_CTRL parameter.
* Change the selection order of barometers using the CAL_BAROx_PRIO parameters for each barometer.


5. Rangefinders

EKF2_RNG_CTRL (INT32)


作用：测量无人机与地面或障碍物的距离。
坐标系：机体系，通常沿 Z 轴（垂直向下）测量，但也可以安装在前/侧，方向依照安装姿态。

Generic Configuration
The common rangefinder configuration is specified using EKF2_RNG_* parameters. These include (non exhaustively):
* EKF2_RNG_POS_X, EKF2_RNG_POS_Y, EKF2_RNG_POS_Z - offset of the rangefinder from the vehicle centre of gravity in X, Y, Z directions.
* EKF2_RNG_PITCH - A value of 0 degrees (default) corresponds to the range finder being exactly aligned with the vehicle vertical axis (i.e. straight down), while 90 degrees indicates that the range finder is pointing forward. Simple trigonometry is used to calculate the distance to ground if a non-zero pitch is used.
* EKF2_RNG_DELAY - approximate delay of data reaching the estimator from the sensor.
* EKF2_RNG_SFE - Range finder range dependent noise scaler.
* EKF2_RNG_NOISE - Measurement noise for range finder fusion



6. GNSS (GPS)
作用提供

ned 坐标下的
Latitude & Longitude
Altitude
Ground Speed (NED velocity components)
Course Over Ground (COG) / Heading  
* Derived from velocity vector (not magnetometer-based yaw)
但是起飞后px4 内部能通过 gps 获得 yaw：it is possible to use post-takeoff horizontal movement combined with GNSS velocity measurements to align the yaw angle.实际就是对比 cog 和imu。


EKF2_GPS_CTRL (INT32)

Bitmask:
* 0: Lon/lat
* 1: Altitude
* 2: 3D velocity
* 3: Dual antenna heading


EKF2_GPS
EKF2_GPS 


7.rtk
同 gps，我们的这款 rtk 也做不了 dual，所以无法提供 yaw
Tuning
You may also need to tune some parameters as the default parameters are tuned assuming a GPS accuracy in the order of meters, not centimeters. For example, you can decrease EKF2_GPS_V_NOISE and EKF2_GPS_P_NOISE to 0.2.

Choose a position where the base module won't need to be moved, has a clear view of the sky, and is well separated from any buildings. Often it is helpful to elevate the base GPS, by using a tripod or mounting it on a roof.

Dual Receivers
A second GPS receiver can be used as a backup！！！！！ (either RTK or non RTK). See the Using PX4's Navigation Filter (EKF2) > GPS section.




9.ev
EKF2_EV_CTRL (INT32)

Bitmask:
* 0: Horizontal position
* 1: Vertical position
* 2: 3D velocity
* 3: Yaw


需要注意的点
* VIO 的 yaw 是相对坐标系的，零点取决于系统启动时的参考方向。
    * 如果要和世界坐标系对齐，通常需要额外的初始化（比如对齐到地图或北向）。
* 纯视觉系统的 yaw 可能在长时间飞行后漂移，尤其是在环境纹理少或光照变化大时。

Parameter	Setting for External Position Estimation
EKF2_EV_CTRL	Set horizontal position fusion, vertical vision fusion, velocity fusion, and yaw fusion according to your desired fusion model.
EKF2_HGT_REF	Set to Vision to use the vision as the reference sensor for altitude estimation.
EKF2_EV_DELAY	Set to the difference between the timestamp of the measurement and the "actual" capture time. For more information see below.
EKF2_EV_POS_X, EKF2_EV_POS_Y, EKF2_EV_POS_Z	Set the position of the vision sensor with respect to the vehicle's body frame.

—————————

关于 yaw 的部分：

EKF2_MAG_TYPE (INT32) Type of magnetometer fusion. Integer controlling the type of magnetometer fusion used - magnetic heading or 3-component vector. The fusion of magnetometer data as a three component vector enables vehicle body fixed hard iron errors to be learned, but requires a stable earth field. If set to 'Automatic' magnetic heading fusion is used when on-ground and 3-axis magnetic field fusion in-flight. If set to 'Magnetic heading' magnetic heading fusion is used at all times. If set to 'None' the magnetometer will not be used under any circumstance. If no external source of yaw is available, it is possible to use post-takeoff horizontal movement combined with GNSS velocity measurements to align the yaw angle. If set to 'Init' the magnetometer is only used to initalize the heading. Values: 0: Automatic 1: Magnetic heading 5: None 6: Init

原话：Note that if yaw data is used (bit 3) the heading is with respect to the external vision frame; otherwise the heading is relative to North.


yaw 来自于磁力计，ev ，gps 也可推断参见上方（但是一定要是运动的），同时双天线 gps 也能提供 yaw、

—————————




Typical configurations
	EKF2_GPS_CTRL	EKF2_BARO_CTRL	EKF2_RNG_CTRL	EKF2_HGT_REF	EKF2_EV_CTRL
Outdoor (default)	7 (Lon/lat/alt/vel)	1 (enabled)	1 (conditional)	1 (vision)	horizontal position，vertical position
Indoor (non-flat terrain)	0 (disabled)	1 (enabled)	1 (conditional)	2 (range)	horizontal position，vertical position,yaw
Indoor (flat terrain)	0 (disabled)	1 (enabled)	2 (always enabled)	2 (range)	horizontal position，vertical position,yaw

————————————
MAVROS will take care of NED conversions.

realsense-viewer

roslaunch realsense2_camera rs_camera.launch



-----------------------
坐标轴:

Depending on the source of your reference frame, you will need to apply a custom transformation to the pose estimate before sending the MAVLink Vision/MoCap message. This is necessary to change the orientation of the parent and child frame of the pose estimate, such that it fits the PX4 convention. Have a look at the MAVROS odom plugin for the necessary transformations.


The MAVROS odometry plugin makes it easy to handle the coordinate frames. It uses ROS's tf package. Your external pose system might have a completely different frame convention that does not match the one of PX4. The body frame of the external pose estimate can depend on how you set the body frame in the MOCAP software or on how you mount the VIO sensor on the drone. The MAVROS odometry plugin needs to know how the external pose's child frame is oriented with respect to either the airframe's FRD or FLU body frame known by MAVROS. You therefore have to add the external pose's body frame to the tf tree. This can be done by including an adapted version of the following line into your ROS launch file.


  <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="0 0 0 <yaw> <pitch> <roll> base_link <external_pose_child_frame> 1000"/>
Make sure that you change the values of yaw, pitch and roll such that it properly attaches the external pose's body frame to the base_link or base_link_frd. Have a look at the tf package for further help on how to specify the transformation between the frames. You can use rviz to check if you attached the frame right. The name of the external_pose_child_frame has to match the child_frame_id of your nav_msgs/Odometry message. The same also applies for the reference frame of the external pose. You have to attach the reference frame of the external pose as child to either the odom or odom_frd frame. Adapt therefore the following code line accordingly.


  <node pkg="tf" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
        args="0 0 0 <yaw> <pitch> <roll> odom <external_pose_parent_frame> 1000"/>
If the reference frame has the z axis pointing upwards you can attached it without any rotation (yaw=0, pitch=0, roll=0) to the odom frame. The name of external_pose_parent_frame has to match the frame_id of the odometry message.
INFO
When using the MAVROS odom plugin, it is important that no other node is publishing a transform between the external pose's reference and child frame. This might break the tf tree.


If you're working with EKF2, only the "vision" pipelines are supported. To use MoCap data with EKF2 you will have to remap the pose topic that you get from MoCap:
* MoCap ROS topics of type geometry_msgs/PoseStamped or geometry_msgs/PoseWithCovarianceStamped must be remapped to /mavros/vision_pose/pose. The geometry_msgs/PoseStamped topic is most common as MoCap doesn't usually have associated covariances to the data.
* If you get data through a nav_msgs/Odometry ROS message then you will need to remap it to /mavros/odometry/out, making sure to update the frame_id and child_frame_id accordingly.
* The odometry frames frame_id = odom, child_frame_id = base_link can be changed by updating the file in mavros/launch/px4_config.yaml. However, the current version of mavros (1.3.0) needs to be able to use the tf tree to find a transform from frame_id to the hardcoded frame odom_ned. The same applies to the child_frame_id, which needs to be connected in the tf tree to the hardcoded frame base_link_frd. If you are using mavros 1.2.0 and you didn't update the file mavros/launch/px4_config.yaml, then you can safely use the odometry frames frame_id = odom, child_frame_id = base_link without much worry.
* Note that if you are sending odometry data to px4 using child_frame_id = base_link, then you need to make sure that the twist portion of the nav_msgs/Odometry message is expressed in body frame, not in inertial frame!!!!!.
—————————

pose → 在 header.frame_id 坐标系下表示（常见：odom 或 map）。
twist → 是 child_frame_id 对应的坐标系中的速度（常见：base_link）。
本身 ros 就是这么设定的
—————————
Bitmask: 0: use GPS 1: use optical flow 2: inhibit IMU bias estimation 3: vision position fusion 4: vision yaw fusion 5: multi-rotor drag fusion 6: rotate external vision 7: GPS yaw fusion 8: vision velocity fusion 

闲鱼的那个人要我设置成，十进制的 24，也就是
bit3: vision position fusion
bit4: vision yaw fusion


————————————————
如果你要做室内飞行（没有 GPS，没有磁力计），那么用视觉坐标系的 yaw 完全没问题，因为导航、控制只关心相对位置和角度。 如果你要做室外飞行或需要和地图对齐（比如航点任务），就需要 yaw 相对于真北，这时不能直接用视觉 yaw，而是要用磁力计/GNSS 融合来得到全球参考的航向。 在 px4 里面怎么设定这个 yaw



你理解得很对。在 PX4 里，“用什么来当作航向/参考的 yaw”主要是通过 EKF2 相关参数来选的。下面给你两套常用配置（室内相对 yaw vs. 室外真北对齐），以及参数在哪里改。
室内/只要相对 yaw（视觉系）
用外部视觉/动捕提供的位置/速度/姿态，不依赖磁力计或 GNSS 的绝对航向。
1. 打开外部视觉融合并融合 yaw：
* EKF2_EV_CTRL 打开第 3 位（yaw 数据）。常见取值：
    * 仅融合视觉 yaw：8
    * 融合视觉“水平位姿+高度+速度+yaw”：15（推荐 VIO 全融合）
* 文档写明：若融合了视觉 yaw（置位 bit3），航向将以外部视觉坐标系为参考；若不融合视觉 yaw，则航向相对于“北”。docs.px4.io
1. 关闭磁力计（避免被磁干扰拖偏）：
* EKF2_MAG_TYPE = 5 (none)。PX4 还支持在飞行后通过 GPS 速度观测自对准 yaw，但纯室内通常没 GPS，这里直接关掉磁力计最稳。docs.px4.io
备忘：如需把高度也交给视觉，设置 EKF2_HGT_REF 为 “Vision”。docs.px4.io


室外/需要与地图（真北）对齐
有三种主路子，任选其一（或作冗余）：
A. 常规磁力计航向（最常见）
* 做好指南针校准并保持磁环境干净；保持磁力计融合开启即可（默认会用）。若在强磁环境下易漂，考虑改用 B 或 C。〔官方页面侧重 GPS/视觉阐述，这里是通用实践。〕
B. 双天线 GNSS（Moving Baseline / GPS Yaw） — 精度高、抗磁干扰
* 在 EKF2_GPS_CTRL 打开“Dual antenna heading”位（bit3=1，也就是在当前值上加 8）。
* 设 GPS_YAW_OFFSET 为两天线连线相对于机体 X 轴的角度。
* 若用 u-blox F9P 等方案，按设备文档配置“Moving Base / Rover”模式。docs.px4.io+3docs.px4.io+3docs.px4.io+3
C. 基于 GPS 速度的航向估计（GSF 备份/替代磁罗盘）
* 设 EKF2_MAG_TYPE = 5 (none) 关闭磁力计；起飞后做一定水平运动，EKF 会用 GPS 速度把主滤波的 yaw 对齐。适合磁环境差且不方便上双天线的场景。docs.px4.io



小贴士 / 易错点
* 融合视觉 yaw 时，请确保你发送的 MAVLink ODOMETRY/vision 的姿态定义与 PX4 期望一致（机体系/坐标轴方向一致）；用错坐标系会导致航向整体偏转。文档已强调“融合视觉 yaw 就是以视觉系为参考”。docs.px4.io
* 双天线 GPS 要正确设置天线基线方向与 GPS_YAW_OFFSET；装配方向错 90°/180° 会让航向恒偏。docs.px4.io+1
* 若关闭磁力计（MAG_TYPE=5）而又没有视觉 yaw，且没有 GPS 速度信息（例如室内静止），yaw 会不可观测或缓慢漂移 —— 这属于算法可观测性限制。GitHub



——————————

用不上先不用看：
Collision Prevention
Precision Landing



remote controller：
遥控器右下方自带 gps


uart0连接了 telem1
uarto和uart1的波特率不一样

吊舱，或者附加的云台要调整到这个网段才可以工作192.168.144


视频传输

只要支持 rtsp 播放就可以查看视频
qgc 选视频来源

联网方式:

- [ ] 只能连5g wifi 
- [ ] 连网卡
- [ ] 网口插有线网络，实测不稳定

数据分享：
- [ ] 开热点 用另一台电脑连接，就能数据分享给另一台电脑，但是一般用不上


检查下： 

configuration tuning 还没看

1.https://docs.px4.io/main/en/config/autotune_mc.html

https://docs.px4.io/main/en/config/safety_intro.html 还没看

https://docs.px4.io/main/en/advanced_config/esc_calibration.html



https://docs.px4.io/main/en/config/battery.html


这个比较复杂：
https://docs.px4.io/main/en/advanced_config/compass_power_compensation.html

https://docs.px4.io/main/en/config/accelerometer.html 
https://docs.px4.io/main/en/config/gyroscope.html




——————————————————————————————————————————

⚠️ 警告：降落问题（虽然极少见）
1. 无人机降不下来或持续水平移动：
* 可切换到“高度模式”进行手动降落，操作方法与上述相同；
* 着陆后检查 GPS 和磁力计方向、校准情况。
2. 无人机触地后未检测到并未解锁：
* 保持油门在最低，切换到自稳模式（Stabilized）；
* 使用姿态手势或命令手动解锁； disarm
* 或者，使用kill switch 杀死电机（仅在飞机已在地面上时使用）。


Multicopters can be landed in any manual mode. Make sure to keep the throttle stick pulled down after touching down until the motors have switched off.

——————————————————————————————————————————
坐标轴:

Depending on the source of your reference frame, you will need to apply a custom transformation to the pose estimate before sending the MAVLink Vision/MoCap message. This is necessary to change the orientation of the parent and child frame of the pose estimate, such that it fits the PX4 convention. Have a look at the MAVROS odom plugin for the necessary transformations.


The MAVROS odometry plugin makes it easy to handle the coordinate frames. It uses ROS's tf package. Your external pose system might have a completely different frame convention that does not match the one of PX4. The body frame of the external pose estimate can depend on how you set the body frame in the MOCAP software or on how you mount the VIO sensor on the drone. The MAVROS odometry plugin needs to know how the external pose's child frame is oriented with respect to either the airframe's FRD or FLU body frame known by MAVROS. You therefore have to add the external pose's body frame to the tf tree. This can be done by including an adapted version of the following line into your ROS launch file.


  <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
        args="0 0 0 <yaw> <pitch> <roll> base_link <external_pose_child_frame> 1000"/>
Make sure that you change the values of yaw, pitch and roll such that it properly attaches the external pose's body frame to the base_link or base_link_frd. Have a look at the tf package for further help on how to specify the transformation between the frames. You can use rviz to check if you attached the frame right. The name of the external_pose_child_frame has to match the child_frame_id of your nav_msgs/Odometry message. The same also applies for the reference frame of the external pose. You have to attach the reference frame of the external pose as child to either the odom or odom_frd frame. Adapt therefore the following code line accordingly.


  <node pkg="tf" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
        args="0 0 0 <yaw> <pitch> <roll> odom <external_pose_parent_frame> 1000"/>
If the reference frame has the z axis pointing upwards you can attached it without any rotation (yaw=0, pitch=0, roll=0) to the odom frame. The name of external_pose_parent_frame has to match the frame_id of the odometry message.
INFO
When using the MAVROS odom plugin, it is important that no other node is publishing a transform between the external pose's reference and child frame. This might break the tf tree.


If you're working with EKF2, only the "vision" pipelines are supported. To use MoCap data with EKF2 you will have to remap the pose topic that you get from MoCap:
* MoCap ROS topics of type geometry_msgs/PoseStamped or geometry_msgs/PoseWithCovarianceStamped must be remapped to /mavros/vision_pose/pose. The geometry_msgs/PoseStamped topic is most common as MoCap doesn't usually have associated covariances to the data.
* If you get data through a nav_msgs/Odometry ROS message then you will need to remap it to /mavros/odometry/out, making sure to update the frame_id and child_frame_id accordingly.
* The odometry frames frame_id = odom, child_frame_id = base_link can be changed by updating the file in mavros/launch/px4_config.yaml. However, the current version of mavros (1.3.0) needs to be able to use the tf tree to find a transform from frame_id to the hardcoded frame odom_ned. The same applies to the child_frame_id, which needs to be connected in the tf tree to the hardcoded frame base_link_frd. If you are using mavros 1.2.0 and you didn't update the file mavros/launch/px4_config.yaml, then you can safely use the odometry frames frame_id = odom, child_frame_id = base_link without much worry.
* Note that if you are sending odometry data to px4 using child_frame_id = base_link, then you need to make sure that the twist portion of the nav_msgs/Odometry message is expressed in body frame, not in inertial frame!!!!!.
—————————


天天搁我yaw estimate error，我寻思着这也没有任何电线从飞控上方经过，一直校准那一堆传感器，把COM_ARM_EKF_YAW改到1也还是那样，不如直接禁用罗盘得了

设置参数：

CAL_MAG0_PRIO 设置为Disabled。？？？？？？

SYS_HAS_MAG=0

EKF2_MAG_TYPE=5(NONE)


————————————————



如果你要做室内飞行（没有 GPS，没有磁力计），那么用视觉坐标系的 yaw 完全没问题，因为导航、控制只关心相对位置和角度。 如果你要做室外飞行或需要和地图对齐（比如航点任务），就需要 yaw 相对于真北，这时不能直接用视觉 yaw，而是要用磁力计/GNSS 融合来得到全球参考的航向。 在 px4 里面怎么设定这个 yaw



你理解得很对。在 PX4 里，“用什么来当作航向/参考的 yaw”主要是通过 EKF2 相关参数来选的。下面给你两套常用配置（室内相对 yaw vs. 室外真北对齐），以及参数在哪里改。
室内/只要相对 yaw（视觉系）
用外部视觉/动捕提供的位置/速度/姿态，不依赖磁力计或 GNSS 的绝对航向。
1. 打开外部视觉融合并融合 yaw：
* EKF2_EV_CTRL 打开第 3 位（yaw 数据）。常见取值：
    * 仅融合视觉 yaw：8
    * 融合视觉“水平位姿+高度+速度+yaw”：15（推荐 VIO 全融合）
* 文档写明：若融合了视觉 yaw（置位 bit3），航向将以外部视觉坐标系为参考；若不融合视觉 yaw，则航向相对于“北”。docs.px4.io
1. 关闭磁力计（避免被磁干扰拖偏）：
* EKF2_MAG_TYPE = 5 (none)。PX4 还支持在飞行后通过 GPS 速度观测自对准 yaw，但纯室内通常没 GPS，这里直接关掉磁力计最稳。docs.px4.io
备忘：如需把高度也交给视觉，设置 EKF2_HGT_REF 为 “Vision”。docs.px4.io


室外/需要与地图（真北）对齐
有三种主路子，任选其一（或作冗余）：
A. 常规磁力计航向（最常见）
* 做好指南针校准并保持磁环境干净；保持磁力计融合开启即可（默认会用）。若在强磁环境下易漂，考虑改用 B 或 C。〔官方页面侧重 GPS/视觉阐述，这里是通用实践。〕
B. 双天线 GNSS（Moving Baseline / GPS Yaw） — 精度高、抗磁干扰
* 在 EKF2_GPS_CTRL 打开“Dual antenna heading”位（bit3=1，也就是在当前值上加 8）。
* 设 GPS_YAW_OFFSET 为两天线连线相对于机体 X 轴的角度。
* 若用 u-blox F9P 等方案，按设备文档配置“Moving Base / Rover”模式。docs.px4.io+3docs.px4.io+3docs.px4.io+3
C. 基于 GPS 速度的航向估计（GSF 备份/替代磁罗盘）
* 设 EKF2_MAG_TYPE = 5 (none) 关闭磁力计；起飞后做一定水平运动，EKF 会用 GPS 速度把主滤波的 yaw 对齐。适合磁环境差且不方便上双天线的场景。docs.px4.io



小贴士 / 易错点
* 融合视觉 yaw 时，请确保你发送的 MAVLink ODOMETRY/vision 的姿态定义与 PX4 期望一致（机体系/坐标轴方向一致）；用错坐标系会导致航向整体偏转。文档已强调“融合视觉 yaw 就是以视觉系为参考”。docs.px4.io
* 双天线 GPS 要正确设置天线基线方向与 GPS_YAW_OFFSET；装配方向错 90°/180° 会让航向恒偏。docs.px4.io+1
* 若关闭磁力计（MAG_TYPE=5）而又没有视觉 yaw，且没有 GPS 速度信息（例如室内静止），yaw 会不可观测或缓慢漂移 —— 这属于算法可观测性限制。GitHub

——————————


以 /mavros/odometry/out 消息为例，假设：

header.frame_id: "odom"
child_frame_id: "base_link"
你可以这样理解：
这是一个从 "odom" 坐标系（全局参考系）到 "base_link" 坐标系（无人机本体）的转换。


base_link 是 ROS 中常用的一个标准坐标系名称，表示机器人（或无人机、自动车等）本体的参考坐标系。它是描述机器人的“自身姿态和位置”的核心坐标框架。它通常定义在机器人底盘中心、几何中心或质心附近的位置。


* The odometry frames frame_id = odom, child_frame_id = base_link can be changed by updating the file in mavros/launch/px4_config.yaml. However, the current version of mavros (1.3.0) needs to be able to use the tf tree to find a transform from frame_id to the hardcoded frame odom_ned. The same applies to the child_frame_id, which needs to be connected in the tf tree to the hardcoded frame base_link_frd. If you are using mavros 1.2.0 and you didn't update the file mavros/launch/px4_config.yaml, then you can safely use the odometry frames frame_id = odom, child_frame_id = base_link without much worry.
？

MAVROS will take care of NED conversions.

local body frame
reference frame
FRD 也叫NED