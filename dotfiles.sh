git clone --bare git@github.com:kennyjchen/dotfiles.git $HOME/.dotfiles
/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME checkout
