# default bashrc
source ~/.bashrc_default

# vim bindings
set -o vi

# fix for ctrl-l in bash vi mode
bind -m vi-command 'Control-l: clear-screen'
bind -m vi-insert 'Control-l: clear-screen'

# shorten path
PROMPT_DIRTRIM=1

# disable ctrl-s
if [[ -t 0 && $- = *i* ]]
then
    stty -ixon
fi 

# bash aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias vb='vim ~/.bashrc'

mkvideo() {
  ffmpeg -i $1 -filter:v "setpts=PTS/$2" -vcodec libx264 -crf 32 $3
}

# ros stuff
source /opt/ros/noetic/setup.bash
source /home/kjchen/Software/Workspaces/main/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

alias dlio='roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true'
alias magicroute='sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.5'
alias rosmaster_aquila='export ROS_MASTER_URI=http://192.168.2.1:11311 && export ROS_IP=192.168.1.101'
alias rosmaster_local='export ROS_MASTER_URI=http://localhost:11311'

alias cc='catkin clean --this'
alias cca='catkin clean'
alias cb='catkin build --this'
alias cba='catkin build'
alias sws='source devel/setup.bash'

dlio_savemap() {
  rosservice call /robot/dlio_map/save_pcd $1 /home/kjchen/Downloads
}

rbp() {
  if [[ "$1" == "dlio" ]]; then
    rosbag play $2 /aquila1/os_cloud_node/points:=/robot/lidar /aquila1/mpu6050/imu:=/robot/imu $3
  elif [[ "$1" == "liosam" ]]; then
    rosbag play $2 /points_raw:=/robot/lidar /imu_correct:=/robot/imu $3
  elif [[ "$1" == "kitti" ]]; then
    rosbag play $2 /kitti/velo/pointcloud:=/robot/lidar /kitti/oxts/imu:=/robot/imu $3
  elif [[ "$1" == "newer2020_ouster" ]]; then
    rosbag play $2 /os1_cloud_node/points:=/robot/lidar /os1_cloud_node/imu:=/robot/imu $3
  elif [[ "$1" == "newer2020_rs" ]]; then
    rosbag play $2 /os1_cloud_node/points:=/robot/lidar /camera/imu:=/robot/imu $3
  elif [[ "$1" == "newer2021" ]]; then
    rosbag play $2 /os_cloud_node/points:=/robot/lidar /alphasense_driver_ros/imu:=/robot/imu $3
  fi
}

# miniconda
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

