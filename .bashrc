# default bashrc
source ~/.bashrc_default

# additional stuff
source ~/.bashrc_fieldai
alias sfai='source ~/.bashrc_fieldai'

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
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# bash aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias cdlast='cd "$(ls -d */ | tail -n 1)"'

alias cloudcompare='prime-run flatpak run org.cloudcompare.CloudCompare'

# ros stuff
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

export ROS_WORKSPACE=~/Projects/ros_ws/src
source $ROS_WORKSPACE/../devel/setup.bash

alias use_sim_time="rosparam set use_sim_time true"
alias plotjuggler="rosrun plotjuggler plotjuggler"

alias cc='catkin clean --this'
alias cca='catkin clean'
alias cb='catkin build --this'
alias cba='catkin build'
alias sws='source devel/setup.bash'

alias cdp='cd /home/kjchen/Projects/ros_ws'
alias cdd='cd /home/kjchen/Projects/ros_ws/src/robot-monorepo/dlio_v2'
alias cdt='cd /media/kjchen/T9'

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
