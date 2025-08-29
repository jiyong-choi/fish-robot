# Paste it in the bottom of the ~/.bashrc file

# Shared ROS env from ~/.ros_env.
# Edit ros_env to change ROS_MASTER_IP
set -a
[ -f "$HOME/.ros_env" ] && . "$HOME/.ros_env"
set +a
export ROS_MASTER_URI="http://$ROS_MASTER_IP:11311"
export ROS_IP=$(hostname -I | awk '{print $1}')

# Source for ROS settings
source /opt/ros/noetic/setup.bash
