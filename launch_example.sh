#!/bin/bash

SESSION_NAME="gz_sim_dev_tmux"
tmux new-session -d -s $SESSION_NAME

tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 1
# tmux split-window -v
# tmux select-pane -t 2
# tmux split-window -v

tmux send-keys -t 0 "cd && source ros2_ws/install/setup.bash && ros2 launch vehicle_bringup unirobot.launch.py "
tmux send-keys -t 1 "source ~/other_ws/install/setup.bash && ros2 launch traversability_mapping_ros global_gt_traversability_mapping.launch.py"
tmux send-keys -t 2 "cd && cd ros2_ws && colcon build --symlink-install && cd ~/other_ws && colcon build --symlink-install" C-m
tmux send-keys -t 3 "source ~/other_ws/install/setup.bash && ros2 launch traversability_mapping_ros rviz.launch.py"

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME
