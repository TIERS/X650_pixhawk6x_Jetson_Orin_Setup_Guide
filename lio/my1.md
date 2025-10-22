# X650 Setup

## Drone

- **Wi-Fi**：`TIARIAS`
- **SSH**：`ssh x650@192.168.50.240`  
  **Password**：`future`


# JetPack 5.1.2  ros1 Noetic

## Sensors





### realsense Configuration

realsense D435

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md


串口：

“UART1” 指的是 Jetson 主板上的第一个通用异步收发传输器（Universal Asynchronous Receiver/Transmitter）接口，也就是串口中的一个硬件控制单元。你可以把它理解为主板上标号为1的串口控制器。


but!!!!!在我板子上实际连接的是/dev/ttyTHS0

await drone.connect(system_address="serial:///dev/ttyTHS0:921600")



若你追求 高速稳定、多设备互联，推荐用 以太网 + UDP 方式连接 Pixhawk。

Python代码：
mav_x_mode 在 normal 模式下，使用Server 模式
await drone.connect(system_address="udp://:14540")

在 onboard模式下，只能使用Client 模式，主动连接
await drone.connect(system_address="udp://192.168.0.3:14540")


ros驱动：实际就是ros把mavlink包起来了
-----------------------------------------
先用一个终端建立连接：之后可以立马关闭  有时又用不上
echo "hi" | nc -u 192.168.0.3 14540 -p 14540

roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540


-----------------------------------------




I am using a Holybro X650 drone, equipped with a Pixhawk-Jetson baseboard, Pixhawk 6X, and two NVIDIA Jetson Orin NX 16GB modules. The Jetson is connected to a Livox MID360. Jetson also has JetPack 5.1.2 installed.



The Jetson baseboard has a wireless network card, and my development computer (a Dell running Ubuntu 20.04) is connected to the same Wi-Fi as the Jetson. My computer is now connected to the Jetson via SSH.

i have a NVIDIA Jetson Orin NX 16GB. It has JetPack 5.1.2 and ros1 Noetic


echo $LD_LIBRARY_PATH  

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH: 把 /usr/local/lib 加到系统的 LD_LIBRARY_PATH 环境变量的最前面，用于运行时查找动态链接库。


echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
的作用是 永久设置 LD_LIBRARY_PATH 环境变量，以确保系统运行程序时能找到你安装在 /usr/local/lib 目录下的动态库（例如 OpenCV4）。

ldd ~/catkin_ws/devel/lib/librealsense2_camera.so | grep opencv


是在 Linux 系统中用来查看一个共享库（.so 文件）依赖哪些其他共享库的命令。


git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard.html#ethernet-setup-using-netplan



这台6x上的px4的默认ip仍然是192.168.0.3

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

pip3 install mavsdk 就是要等很久，正常


# PX4 Flight Modes Summary

PX4 supports two main types of flight modes: **Manual** and **Autonomous**.

---

## 🕹️ Manual Modes

### ✅ Manual-Easy Modes

#### 1. **Position Mode**
- Easiest and safest manual mode (requires GPS).
- Controls:
  - Roll/Pitch: Ground acceleration (like a car pedal).
  - Yaw: Rotation.
  - Throttle: Climb/descent rate.
- Behavior:
  - Releasing the sticks causes the vehicle to level, brake, and hold position (even against wind).

#### 2. **Position Slow Mode**
- A speed- and yaw-rate-limited version of Position Mode.
- Used for flying near obstacles or for regulatory compliance.

#### 3. **Altitude Mode**
- Easiest manual mode without GPS.
- Releasing sticks:
  - Maintains level and altitude.
  - No horizontal position hold — vehicle drifts with inertia and wind.

#### 4. **Stabilized Mode also Manual Mode**  
- Releasing sticks:
  - Maintains level attitude only (not position or altitude).
  - Subject to drift due to inertia and wind.

---

### 🎯 Manual-Acrobatic Mode

#### 5. **Acro Mode**
- Designed for acrobatic maneuvers (e.g., rolls, flips).
- Releasing sticks stops rotation, but does not auto-level or stabilize.

---

## 🤖 Autonomous Modes

### ✅ Fully Autonomous

#### 1. **Hold**
- Vehicle hovers in place, holding both position and altitude.

#### 2. **Return (RTL)**
- Ascends to a safe altitude, returns to home or rally point, then lands.
- Requires GPS.

#### 3. **Mission**
- Executes a pre-uploaded mission (waypoints).
- Requires GPS.

#### 4. **Takeoff**
- Performs vertical takeoff, then switches to Hold.

#### 5. **Land**
- Lands immediately.

#### 6. **Orbit**
- Circles a point while yawing to face the center.
- RC control can adjust radius, speed, and direction.

#### 7. **Follow Me**
- Follows a beacon transmitting position setpoints (e.g., phone/GCS).
- RC can configure the follow offset.

#### 8. **Offboard**
- Obeys position, velocity, or attitude setpoints sent via MAVLink or ROS 2.

---

## 🔄 Flight Mode Switching

- Modes are switched using:
  - RC transmitter switches, or
  - Ground Control Station (e.g., QGroundControl).

> PX4 will **not allow switching** to certain modes unless preconditions are met:
- GPS lock,
- Airspeed sensor available,
- Valid vehicle orientation, etc.

---

For detailed information, refer to the mode-specific sidebar topics in the [PX4 documentation](https://docs.px4.io/).


jetson主机ip：192.168.0.1
mid360ip：192.168.0.2
fc ip：192.168.0.3

I/O PWM out= main out
FMU pwm out= aux 连接的马达
telem2 内部已经连接到 jetson,我的实验里面不上


mavlink has 3 instances.
mav_0 is in telem1, connected to skydriod rc
mav_1 is in telem3, connect to telemetry radio
mav_2 is in ethernet, connect to switch on jetsonboard, so it's onboard mode.

px4版本：1.15.4

RC IN:遥控器
i2c连接了range finder

qgc4.4.4版本 https://docs.qgroundcontrol.com/Stable_V4.4/en/


safety switch在 gps 上，我这个应该关掉了

An arming button or "momentary switch" can be configured to trigger arm/disarm instead of gesture-based arming (setting an arming switch disables arming gestures). The button should be held down for (nominally) one second to arm (when disarmed) or disarm (when armed).


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
Publish pointcloud2 format data
Autoload rviz


msg_MID360.launch	Connect to MID360 LiDAR device
Publish livox customized pointcloud data




pool ntp.ubuntu.com        iburst maxsources 4
pool 0.ubuntu.pool.ntp.org iburst maxsources 1
pool 1.ubuntu.pool.ntp.org iburst maxsources 1
pool 2.ubuntu.pool.ntp.org iburst maxsources 2

keyfile /etc/chrony/chrony.keys

allow 192.168.1.0/24

local stratum 8


driftfile /var/lib/chrony/chrony.drift

#log tracking measurements statistics
logdir /var/log/chrony

maxupdateskew 100.0

# real-time clock. Note that it can’t be used along with the 'rtcfile' directive.
hwclockfile /etc/adjtime
rtcsync

manual
makestep 0.1 3

--------------------------------


server 192.168.1.225 iburst
makestep 0.1 3


--------------------------------
# 手动设准时间（示例）
sudo timedatectl set-time "2025-10-02 14:36:00"
sudo hwclock --systohc


# 查看自身状态

chronyc clients 里能看到来自各客户端 IP 的请求计数。

chronyc tracking 会显示本机时钟漂移估计、当前 Stratum（应为 8）、同步源（无上游时会显示本地）。

在 客户端 上：

# 源列表与选择情况（带标志位）
chronyc sources -v

# 跟踪状态（偏差、抖动、stratum、上次更新时间等）

# 活动状态（已知多少源）
chronyc activity

timedatectl

         # 看偏差与稳
date --iso-8601=ns       # 两机各打印一次，肉眼对比

sudo systemctl daemon-reload
sudo systemctl enable chrony
sudo systemctl restart chrony


systemctl status chrony

Minimum Return Altitude
By default the minimum return altitude is set using RTL_RETURN_ALT, and the vehicle will just return at the higher of RTL_RETURN_ALT or the initial vehicle altitude.
The minimum return altitude can be further configured using RTL_CONE_ANG, which together with RTL_RETURN_ALT defines a half cone centered around the destination landing point. The cone angle allows a lower minimum return altitude when the return mode is executed close to the destination. This is useful when there are few obstacles near the destination, because it may reduce the minimum height that the vehicle needs to ascend before landing, and hence power consumption and time to land.
￼
The cone affects the minimum return altitude if return mode is triggered within the cylinder defined by the maximum cone radius and RTL_RETURN_ALT: outside this cyclinder RTL_RETURN_ALT is used. Inside the code the minimum return altitude is the intersection of the vehicle position with the cone, or RTL_DESCEND_ALT (whichever is higher). In other words, the vehicle must always ascend to at least RTL_DESCEND_ALT if below that value.
For more information on this return type see Home/Rally Point Return Type (RTL_TYPE=0)


芬兰使用 低功率下433，或者 868，不能使用 915



每次无人机放在原地，所有软件打开后再飞，稳定 5 分钟

indoor：直接只依赖于 lio 关闭磁力计和 gpsEKF2_EV_CTRL ：horizontal position，vertical position，yaw ;yaw will use ev as reference
EKF2_MAG_TYPE：none 
EKF2_HGT_REF：vision
EKF2_gps_CTRL：first three closed
SYS_HAS_MAG=0
SYS_HAS_gps：disabled

CAL_MAG0_PRIO:Disabled(medium)
CAL_MAG1_PRIO:disabled(high)
CAL_MAG2_PRIO:disabled(high)


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
这个几个参数要设置一下：距离 cog 的距离
In order to compensate for the relative motion between the receiver and the CoG, you should configure the following parameters to set the offsets: EKF2_GPS_POS_X, EKF2_GPS_POS_Y and EKF2_GPS_POS_Z.
This is important because the body frame estimated by the EKF will converge on the location of the GNSS module and assume it to be at the CoG. If the GNSS module is significantly offset from the CoG, then rotation around the COG will be interpreted as an altitude change, which in some flight modes (such as position mode) will result in unnecessary corrections.
It is particularly important if using RTK GNSS which has centimeter-level accuracy, because if the offsets are not set then GNSS measurements will often be rejected as inconsistent with the current EFK estimate.


The common rangefinder configuration is specified using EKF2_RNG_* parameters. These include (non exhaustively):
* EKF2_RNG_POS_X, EKF2_RNG_POS_Y, EKF2_RNG_POS_Z - offset of the rangefinder from the vehicle centre of gravity in X, Y, Z directions.


———————————
装了 新的payload之后，do sensors calibration and auto tuning
我是把gps那个底座的平面当做了重心

在 LIO SLAM算法启动时，会把第一帧相机位置设为坐标原点，前向/右向/下向作为 X/Y/Z 轴。


————————————
在 LIO SLAM算法启动时，会把第一帧相机位置设为坐标原点，前向/右向/下向作为 X/Y/Z 轴。
————————————————————

坐标系


PX4 uses FRD (X Forward, Y Right and Z Down) for the local body frame as well for the reference frame. When using the heading of the magnetometer, the PX4 reference frame x axis will be aligned with north, so therefore it is called NED (X North, Y East, Z Down).

Frame	PX4	ROS
Body	FRD (X Forward, Y Right, Z Down)	FLU (X Forward, Y Left, Z Up), usually named base_link
World	FRD or NED (X North, Y East, Z Down)	FLU or ENU (X East, Y North, Z Up), with the naming being odom or map


PX4 sets the NED origin at the first valid global position fix (GPS lat/lon/alt). If no GPS is available, it uses the arming position as the origin. All local navigation and position control are expressed relative to this fixed reference.


Yaw（偏航）：指的是无人机 机头相对于地理坐标系（通常是 NED，North-East-Down）北方向的转角。


————————————————————


1. Accelerometer（加速度计） 内置
作用：测量无人机在 机体系 (body frame) 下的线性加速度 (m/s²)。
坐标系：机体系
X 轴：机头指向前
Y 轴：机翼右侧
Z 轴：垂直向下（右手坐标系，NED frame 标准）。

2. Gyroscope（陀螺仪） 内置
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

