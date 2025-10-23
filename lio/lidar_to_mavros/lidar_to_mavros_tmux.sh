#!/usr/bin/env bash
set -euo pipefail

SESSION="px4_lidar_session"


if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi


tmux new-session -d -s "$SESSION" -n main \
  "bash -lc 'roslaunch mavros px4.launch fcu_url:=udp://192.168.0.1:14540@192.168.0.3:14540; exec bash'"


tmux split-window -h -t "$SESSION:0.0" \
  "bash -lc 'sleep 25; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash'"


tmux split-window -v -t "$SESSION:0.0" \
  "bash -lc 'sleep 35; roslaunch fast_lio mapping_mid360.launch; exec bash'"


tmux split-window -v -t "$SESSION:0.1" \
  "bash -lc 'sleep 45; rosrun lidar_to_mavros lidar_to_mavros; exec bash'"


tmux split-window -v -t "$SESSION:0.2" \
  "bash -lc 'sleep 50; rosrun mavros mavcmd long 511 32 8000 0 0 0 0 0; echo DONE; exec bash'"


tmux select-layout -t "$SESSION:0" tiled



tmux attach -t "$SESSION"