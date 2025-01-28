# default bashrc
source ~/.bashrc_default

source /opt/intel/oneapi/vtune/latest/env/vars.sh
source /opt/ros/kilted/setup.bash
source /home/kjchen/Projects/ros2_ws/install/setup.bash

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
alias xhost='xhost +local:root'
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias cdlast='cd "$(ls -d */ | tail -n 1)"'
alias cdd="cd /home/kjchen/Projects/ros2_ws/src/dlio_v2"

alias cloudcompare="prime-run flatpak run org.cloudcompare.CloudCompare"

# virtualenv
export PATH=~/.local/bin:$PATH
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Projects
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh

# Field AI
alias fai_download="/home/kjchen/data/scripts/fai_download.sh"
alias fai_pull="/home/kjchen/data/scripts/fai_pull.sh"
alias fai_payloads="/home/kjchen/data/scripts/fai_payloads.sh"

alias roscd="cd /home/kjchen/fieldai/fieldai-monorepo/ros1_ws/src"
alias roscd2="cd /home/kjchen/fieldai/fieldai-monorepo/ros2_ws/src"

function aws_dlio_download() {
  if [ -z "$1" ]; then
    echo "Usage: aws_dlio_download <s3_path>"
    return 1
  fi

  local s3_path="$1"
  aws s3 sync "$s3_path" . --exclude="*" --include="*lidar*" --include="*state*"
}

function generate_dlio_report() {
  if [ -z "$1" ]; then
    echo "Usage: generate_dlio_report <path_to_dlio>"
    return 1
  fi

  local log_path="$1"
  python3 /home/kjchen/fieldai/fieldai-monorepo/ros2_ws/src/clam/localization/dlio_v2/scripts/python/generate_dlio_report.py "$log_path" --output "/home/kjchen/Downloads/dlio_report.pdf"
}
