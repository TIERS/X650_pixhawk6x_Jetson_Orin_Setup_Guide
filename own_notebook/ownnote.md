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

—————————————

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


————————
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

—————————————

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



