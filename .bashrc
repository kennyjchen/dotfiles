# default bashrc
source ~/.bashrc_default

# shorten path
PROMPT_DIRTRIM=1

# aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias sb='source ~/.bashrc'
alias cb='catkin build'
alias vim='nvim'

# disable Ctrl-s
if [[ -t 0 && $- = *i* ]]
then
    stty -ixon
fi 

# ROS workspaces
source /opt/ros/noetic/setup.bash
source /home/kjchen/Software/Workspaces/dlio/devel/setup.bash

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

