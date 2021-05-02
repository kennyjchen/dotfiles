git clone --bare git@github.com:kennyjchen/dotfiles.git $HOME/.dotfiles
/usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME checkout

# Install ThirdParty Software
mkdir ~/Software/
mkdir ~/Software/ThirdParty/
cd ~/Software/ThirdParty

# Universal Ctags
git clone https://github.com/universal-ctags/ctags.git
cd ./ctags
./autogen.sh
./configure --prefix=/home/kjchen/.local
make
make install

# Virtualenv