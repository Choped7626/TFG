#!/bin/bash

git clone https://github.com/Choped7626/dotfiles.git 
mkdir -p /root/.config/nvim
cp -r ./dotfiles/.config/nvim /root/.config/nvim
rm -r dotfiles
