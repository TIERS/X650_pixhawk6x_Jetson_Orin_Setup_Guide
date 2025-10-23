#!/usr/bin/env bash
set -euo pipefail


SESSION="try_friday_session"


if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi


tmux new-session -d -s "$SESSION" -n main \
  "bash -lc 'roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540; exec bash'"

tmux split-window -h -t "$SESSION:0.0" \
  "bash -lc 'sleep 25; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash'"

tmux split-window -v -t "$SESSION:0.0" \
  "bash -lc 'sleep 30; roslaunch realsense2_camera rs_camera.launch; exec bash'"


tmux split-window -v -t "$SESSION:0.1" \
  "bash -lc 'sleep 40; rosbag record -O mavros_data_$(date +%Y%m%d_%H%M%S) \
/camera/color/image_raw/compressed \
/livox/imu \
/livox/lidar \
/mavros/altitude \
/mavros/local_position/accel \
/mavros/local_position/odom \
/mavros/local_position/pose \
/mavros/local_position/pose_cov \
/mavros/local_position/velocity_body \
/mavros/local_position/velocity_body_cov \
/mavros/local_position/velocity_local \
/mavros/imu/data \
/mavros/imu/data_raw \
/mavros/imu/diff_pressure \
/mavros/imu/mag \
/mavros/imu/static_pressure \
/mavros/global_position/compass_hdg \
/mavros/global_position/global \
/mavros/global_position/gp_lp_offset \
/mavros/global_position/gp_origin \
/mavros/global_position/local \
/mavros/global_position/raw/fix \
/mavros/global_position/raw/gps_vel \
/mavros/global_position/raw/satellites \
/mavros/global_position/rel_alt \
/mavros/global_position/set_gp_origin \
/mavros/gps_input/gps_input \
/mavros/gps_rtk/rtk_baseline \
/mavros/gps_rtk/send_rtcm \
/mavros/gpsstatus/gps1/raw \
/mavros/gpsstatus/gps1/rtk \
/mavros/gpsstatus/gps2/raw \
/mavros/gpsstatus/gps2/rtk \
/camera/color/camera_info \
/tf \
/tf_static \
/mavros/time_reference; exec bash'"


# 第五个 pane：对 0.2 竖分
tmux split-window -v -t "$SESSION:0.2" \
  "bash -lc 'sleep 42; echo DONE; exec bash'"

# 布局平铺
tmux select-layout -t "$SESSION:0" tiled

# （可选）让 pane 中命令退出时保持窗口（如果不想用 exec bash）
# tmux set-option -t "$SESSION" remain-on-exit on

# 附加到会话
tmux attach -t "$SESSION"