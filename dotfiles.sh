git init --bare $HOME/.dotfiles
git clone --bare https://github.com/kennyjchen/dotfiles.git $HOME/.dotfiles

/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME checkout
dotfiles checkout