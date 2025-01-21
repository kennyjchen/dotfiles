# default bashrc
source ~/.bashrc_default

# additional stuff
source ~/.bashrc_extra

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

# Nvidia
# export __NV_PRIME_RENDER_OFFLOAD=1
# export __GLX_VENDOR_LIBRARY_NAME=nvidia

# bash aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias vb='nvim ~/.bashrc'
alias vbe='nvim ~/.bashrc_extra'
alias vv='nvim ~/.vimrc'
alias vt='nvim ~/.tmux.conf'
alias saud='sudo apt update'
alias saug='sudo apt dist-upgrade'
alias saar='sudo apt autoremove'
alias cdlast='cd "$(ls -d */ | tail -n 1)"'

alias vtune="source /opt/intel/oneapi/setvars.sh && vtune-gui"
alias cloudcompare="prime-run flatpak run org.cloudcompare.CloudCompare"

mkvideo() {
  ffmpeg -i $1 -filter:v "setpts=PTS/$2" -vcodec libx264 -crf $3 $4
}

mkgif() {
  ffmpeg -i $1 -filter_complex "fps=30,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=64[p];[s1][p]paletteuse=dither=bayer" $2
}

halfgif() {
  ffmpeg -i $1 -vf "scale=iw/2:ih/2" $2
}

# ros stuff
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

alias dlo='roslaunch direct_lidar_odometry dlo.launch rviz:=true'
alias dlio='roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true'
alias dliom='roslaunch direct_lidar_inertial_odometry_and_mapping dliom.launch rviz:=true'

alias magicroute='sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.5'
alias magicrouterm='sudo route del -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.5'
alias magicroute2='sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.4'
alias magicroute2rm='sudo route del -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.1.4'
alias rosmaster_aquila='export ROS_MASTER_URI=http://192.168.2.1:11311 && export ROS_IP=192.168.1.101'
alias rosmaster_local='export ROS_MASTER_URI=http://localhost:11311'
alias use_sim_time="rosparam set use_sim_time true"
alias plotjuggler="rosrun plotjuggler plotjuggler"

alias cc='catkin clean --this'
alias cca='catkin clean'
alias cb='catkin build --this'
alias cba='catkin build'
alias sws='source devel/setup.bash'

savemap() {
  rosservice call /robot/dliom_map/save_pcd $1 /home/kjchen/Downloads
}

rbp() {
  if [[ "$1" == "aquila1" ]]; then
    rosbag play $2 /aquila1/os_cloud_node/points:=/robot/lidar /aquila1/mpu6050/imu:=/robot/imu $3
  elif [[ "$1" == "aquila2" ]]; then
    rosbag play $2 /aquila2/os_cloud_node/points:=/robot/lidar /aquila2/os_cloud_node/imu:=/robot/imu $3
  elif [[ "$1" == "liosam" ]]; then
    rosbag play $2 /points_raw:=/robot/lidar /imu_correct:=/robot/imu $3
  elif [[ "$1" == "kitti" ]]; then
    rosbag play $2 /kitti/velo/pointcloud:=/robot/lidar /kitti/oxts/imu:=/robot/imu $3
  elif [[ "$1" == "newer2020" ]]; then
    rosbag play $2 /os1_cloud_node/points:=/robot/lidar /os1_cloud_node/imu:=/robot/imu $3
  elif [[ "$1" == "newer2021" ]]; then
    rosbag play $2 /os_cloud_node/points:=/robot/lidar /alphasense_driver_ros/imu:=/robot/imu $3
  elif [[ "$1" == "hilti" ]]; then
    rosbag play $2 /hesai/pandar:=/robot/lidar /alphasense/imu:=/robot/imu $3
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


# cargo 
export PATH=$PATH:~/.cargo/bin/

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
