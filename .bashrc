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
alias cdlast='cd "$(ls -d */ | tail -n 1)"'

# virtualenv
export PATH=~/.local/bin:$PATH
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Projects
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh

# Field AI
alias fai_download="/home/kjchen/data/scripts/fai_download.sh"
alias roscd="cd /home/kjchen/field-ai/robot-monorepo"
alias cdd="cd /home/kjchen/field-ai/dev-docker-setup"

function generate_dlio_report() {
  if [ -z "$1" ]; then
    echo "Usage: generate_dlio_report <path_to_dlio>"
    return 1
  fi

  local log_path="$1"
  python3 /home/kjchen/field-ai/robot-monorepo/dlio_v2/scripts/python/generate_dlio_report.py "$log_path" -o "/home/kjchen/Downloads/dlio_report.pdf"
}
