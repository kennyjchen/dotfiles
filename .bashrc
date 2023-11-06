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
alias vb='nvim ~/.bashrc'
alias vv='nvim ~/.vimrc'
alias vt='nvim ~/.tmux.conf'
alias saud='sudo apt update'
alias saug='sudo apt dist-upgrade'
alias saar='sudo apt autoremove'

alias cdd='cd /mnt/nvme1n1p1/data'

# exyn aliases
exv() {
  exview exview/All.yaml platforms/Robot$1.yaml
}

exa() {
  exagent rerun/RerunSuperOdom.yaml -D"log: $2" platforms/Robot$1.yaml
}

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

# asdf
. "$HOME/.asdf/asdf.sh"
. "$HOME/.asdf/completions/asdf.bash"

# direnv
eval "$(direnv hook bash)"

# Added by exynai-docker-tools install script on Tue Sep  5 07:27:11 PM EDT 2023
export PATH=$PATH:"/home/kjchen/.exyn/exynai-docker-tools/bin"
source "/home/kjchen/.exyn/exynai-docker-tools/lib/exdock-completion.bash"
export EXDOCK_DEFAULT_PROFILE="jammy"

# exyn environment variables
export EXYN_SANDBOX=1
export EXYN_NET_LOCAL_IF_NAME=wlp0s20f3
export EXYN_LOG_DIR=~/xfiles/

# exbuild
export EXYN_FF_EXBUILD_NINJA=1
export EXYN_FF_EXBUILD_CCACHE=1
export EXYN_FF_EXBUILD_MOLD=1

# Added by exworkspace install script on Wed Sep  6 10:01:24 AM EDT 2023
eval $("/home/kjchen/.exyn/exworkspace/bin/exworkspace" --shellenv)

# Type "ew" to set workspace to most recent, then cd to the source directory
function ew() {
    if [[ -f ~/.exyn/latest_workspace && -n "$(cat ~/.exyn/latest_workspace)" ]]; then
        export EXYN_SANDBOX=1 # disable remote comms
        source $(cat ~/.exyn/latest_workspace)/setup.bash
        exworkspace # cd's to the latest
    fi
}

# Type "eb" to set workspace to most recent, then cd to the build directory
function eb() {
    if [[ -f ~/.exyn/latest_workspace && -n "$(cat ~/.exyn/latest_workspace)" ]]; then
        ew
        mkdir -p $EXYN_BUILD/exyn
        cd $EXYN_BUILD/exyn
    fi
}

# mount s3
alias mnts3='goofys -o ro --endpoint https://s3.us-gov-east-1.amazonaws.com stimpy-logs ~/s3'

# fzf ignore build and dotfiles
export FZF_DEFAULT_COMMAND='rg --files --no-ignore-vcs --hidden -g "!build/*" -g "!.*/" -g "!.*"'

