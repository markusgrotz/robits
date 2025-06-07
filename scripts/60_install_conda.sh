#!/bin/bash -exu

# install conda


if [ -d "$HOME/miniconda3" ]; then
    echo "Miniconda already installed at $HOME/miniconda3"
    exit 0
fi

sudo apt install curl 

TEMP_DIR=$(mktemp -d miniconda_XXXXXXXXXX)
cd $TEMP_DIR

curl -SLO https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod +x Miniconda3-latest-Linux-x86_64.sh 
./Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3

eval "$($HOME/miniconda3/bin/conda shell.bash hook)"

conda init bash zsh
conda config --set auto_activate_base false

#rm -rf "$TEMP_DIR"