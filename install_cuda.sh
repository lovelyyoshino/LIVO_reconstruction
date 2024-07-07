# This file shows how to install cuda v11.2 and cudnn8.1 in Ubuntu 20.04
# I have tried many method, this one works for me

# Steps:
# Remove previous installation if need
# Verify the system has a cuda-capable gpu
# Download and install the nvidia cuda toolkit and cudnn
# Setup environmental variables
# Verify the installation

# Remove previous installation.
# If you don't have previous installation, skip this step
sudo apt-get purge nvidia*
sudo apt remove nvidia-*
sudo rm /etc/apt/sources.list.d/cuda*
sudo apt-get autoremove && sudo apt-get autoclean
sudo rm -rf /usr/local/cuda*


# Verify your gpu is cuda enable check
lspci | grep -i nvidia

# System update
sudo apt-get update
sudo apt-get upgrade

# Install other import packages
sudo apt-get install gcc g++ freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev

# First get the PPA repository driver
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install nvidia driver with dependencies
sudo apt install libnvidia-common-515
sudo apt install nvidia-utils-515
sudo apt install libnvidia-gl-515
sudo apt install nvidia-driver-515

# https://blog.csdn.net/ytusdc/article/details/132404002
mkdir ~/cuda_package
cd ~/cuda_package
# Follow the instruction provided by the nvidia website
wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda_11.7.0_515.43.04_linux.run
sudo sh cuda_11.7.0_515.43.04_linux.run

sudo apt-get install gcc
sudo apt-get install g++

# Setup paths
echo 'export PATH=$PATH:/usr/local/cuda-11.7/bin' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.7/lib64' >> ~/.bashrc
echo 'export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda-11.7/lib64' >> ~/.bashrc
source ~/.bashrc
sudo ldconfig



# Install cuDNN new
# 参考链接：https://developer.nvidia.com/rdp/cudnn-archive
sudo apt-get install libfreeimage3 libfreeimage-dev
wget https://developer.download.nvidia.com/compute/cudnn/9.2.1/local_installers/cudnn-local-repo-ubuntu2004-9.2.1_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2004-9.2.1_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2004-9.2.1/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cudnn-cuda-11
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2


# Finally, you can verify you installation
# But you might need to reboot first to use the command.
#nvidia-smi显示的CUDA版本：这代表了你的NVIDIA驱动内置支持的CUDA版本。这是驱动程序级别的支持，主要影响的是GPU的识别和基础驱动层面的CUDA兼容性。
#nvcc -V显示的CUDA Toolkit版本：这代表了安装在你的系统上的CUDA Toolkit的版本。CUDA Toolkit包括了开发和运行CUDA应用所需的编译器、库和工具。
nvidia-smi
nvcc -V

conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
conda config --set show_channel_urls yes
conda create -n pytorch python=3.9
conda activate pytorch
pip3 install torch==2.0.0 torchvision==0.15.0 torchaudio==2.0.0 --index-url https://mirror.sjtu.edu.cn/pytorch-wheels/cu117


#!/bin/bash

# 创建一个临时Python脚本文件
cat <<EOL > verify_cuda.py
# 验证cuda安装
import torch
print(torch.cuda.is_available())  # 返回True则说明已经安装了cuda
# 验证cuDNN安装
from torch.backends import cudnn
print(cudnn.is_available())  # 返回True说明已经安装了cuDNN
import torch
print(torch.__version__)
print(torch.cuda.is_available())
EOL

# 运行Python脚本
python3 verify_cuda.py

# 删除临时Python脚本文件
rm verify_cuda.py
