# 验证cuda安装

import torch
print(torch.cuda.is_available())  # 返回True则说明已经安装了cuda

# 验证cuDNN安装

from torch.backends import cudnn
print(cudnn.is_available())  # 返回True说明已经安装了cuDNN

import torch
print(torch.__version__)
