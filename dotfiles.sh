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

# pyenv
git clone https://github.com/pyenv/pyenv.git ~/.pyenv
cd ~/.pyenv && src/configure && make -C src

# pyenv-virtualenv
git clone https://github.com/pyenv/pyenv-virtualenv.git $(pyenv root)/plugins/pyenv-virtualenv

# pyenv-virtualenvwrapper
git clone https://github.com/pyenv/pyenv-virtualenvwrapper.git $(pyenv root)/plugins/pyenv-virtualenvwrapper