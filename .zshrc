alias dotfiles='/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME'
alias vim='nvim'

source virtualenvwrapper.sh

# zsh pure
autoload -U promptinit; promptinit
prompt pure

# zsh-autosuggestions
source $(brew --prefix)/share/zsh-autosuggestions/zsh-autosuggestions.zsh

# zsh-syntax-highlighting
source $(brew --prefix)/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh

# The following lines have been added by Docker Desktop to enable Docker CLI completions.
fpath=(/Users/kjchen/.docker/completions $fpath)
autoload -Uz compinit
compinit
# End of Docker CLI completions
