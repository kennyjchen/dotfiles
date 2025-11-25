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
alias xhost='xhost +local:root'
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias cdlast='cd "$(ls -d */ | tail -n 1)"'

alias cloudcompare="prime-run flatpak run org.cloudcompare.CloudCompare"

# virtualenv
export PATH=~/.local/bin:$PATH
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Projects
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source ~/.local/bin/virtualenvwrapper.sh

# pyenv
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

# pyenv-virtualenvwrapper
export PYENV_VIRTUALENVWRAPPER_PREFER_PYVENV="true"
eval "$(pyenv virtualenvwrapper -)"
