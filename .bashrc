# default bashrc
source ~/.bashrc_default

# vim bindings
set -o vi

# shorten path
PROMPT_DIRTRIM=1

# disable Ctrl-s
if [[ -t 0 && $- = *i* ]]
then
    stty -ixon
fi 

# bash aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias sb='source ~/.bashrc'
alias vim='nvim'

# ros aliases
alias cc='catkin clean'
alias cb='catkin build --this'
alias cba='catkin build'

alias dlio='roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true'
alias exportmap='rosservice call /robot/dlio_map/export_map 0.01 /home/kjchen/Downloads'

alias magicroute='sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.5'
alias rosmaster_aquila='export ROS_MASTER_URI=http://192.168.2.1:11311 && export ROS_IP=192.168.1.101'
alias rosmaster_localhost='export ROS_MASTER_URI=http://localhost:11311'

rbp() {
  if [[ "$1" == "dlio" ]]; then
    rosbag play "$2" /aquila1/os_cloud_node/points:=/robot/lidar /aquila1/mpu6050/imu:=/robot/imu
  elif [[ "$1" == "liosam" ]]; then
    rosbag play "$2" /points_raw:=/robot/lidar /imu_correct:=/robot/imu
  elif [[ "$1" == "kitti" ]]; then
    rosbag play "$2" /kitti/velo/pointcloud:=/robot/lidar /kitti/oxts/imu:=/robot/imu
  elif [[ "$1" == "newer2020" ]]; then
    rosbag play "$2" /os1_cloud_node/points:=/robot/lidar /os1_cloud_node/imu:=/robot/imu
  elif [[ "$1" == "newer2021" ]]; then
    rosbag play "$2" /os_cloud_node/points:=/robot/lidar /alphasense_driver_ros/imu:=/robot/imu
  fi
}

# ROS workspaces
source /opt/ros/noetic/setup.bash
source /home/kjchen/Software/Workspaces/dlio/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/kjchen/.miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/kjchen/.miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/kjchen/.miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/kjchen/.miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

# virtualenv wrapper
export PATH=~/.local/bin:$PATH
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Software/Projects
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source ~/.local/bin/virtualenvwrapper.sh

# pyenv
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

# pyenv-virtualenvwrapper
export PYENV_VIRTUALENVWRAPPER_PREFER_PYVENV="true"
eval "$(pyenv virtualenvwrapper -)"

