cd /home/sunrise/cxk/lite3_rbnx_ws/primitives/lite3_quadruped/src/Lite3_ROS
source install/setup.bash

ros2 launch transfer transfer_launch.py	use_sim_time:=false		# Launch the related nodes

