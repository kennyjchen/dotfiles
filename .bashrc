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

# bash aliases
alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'
alias sb='source ~/.bashrc'
alias vb='nvim ~/.bashrc'
alias vv='nvim ~/.vimrc'
alias vt='nvim ~/.tmux.conf'
alias saud='sudo apt update'
alias saug='sudo apt dist-upgrade'
alias saar='sudo apt autoremove'

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
