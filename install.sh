# !/bin/bash
sudo apt-get update
sudo apt-get --fix-broken install
sudo apt-get install wget
sudo apt-get install git
sudo apt-get install mlocate
# install cuda toolkit and nvidia-prime
# apt-get install nvidia-cuda-dev nvidia-cuda-toolkit nvidia-nsight nvidia-prime
# install git, SuiteSparse, Lapack, BLAS etc
sudo apt-get install libssl-dev libsuitesparse-dev liblapack-dev libblas-dev libgtk2.0-dev pkg-config libopenni-dev libusb-1.0-0-dev wget zip clang
sudo  apt-get update
mkdir -p ~/third_party_for_slam
cd ~/third_party_for_slam

# Build Cmake
# apt autoremove cmake
# wget https://github.com/Kitware/CMake/releases/download/v3.26.5/cmake-3.26.5.tar.gz
# tar -xvf cmake-3.26.5.tar.gz
# cd cmake-3.26.5
# ./bootstrap
# make -j16
# make install
# cd ..

## 安装Pangolin
wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip
unzip v0.6.zip && cd Pangolin-0.6
mkdir build && cd build
cmake -DCPP11_NO_BOOST=1 ..
make -j8
sudo make install
cd ../../

# Build gflags
git clone https://github.com/gflags/gflags.git
cd gflags
git checkout v2.2.2
mkdir -p build/ && cd build
cmake -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON -DINSTALL_HEADERS=ON -DINSTALL_SHARED_LIBS=ON -DINSTALL_STATIC_LIBS=ON .. && make -j16
sudo make install
cd ../../

# Build glog
git clone https://github.com/google/glog.git
cd glog
git checkout v0.4.0
mkdir build && cd build
cmake .. && make -j16
sudo make install
cd ../../

# Install Eigen 3.3.7
git clone https://github.com/eigenteam/eigen-git-mirror
#安装
cd eigen-git-mirror
git checkout 3.3.7
mkdir build && cd build
cmake .. && sudo make install
#安装后,头文件安装在/usr/local/include/eigen3/
#移动头文件
cp -r /usr/local/include/eigen3/Eigen /usr/local/include 
cd ../../


# Build Ceres
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout 1.14.0
mkdir -p build/ && cd build/
cmake .. && make -j16
sudo make install
cd ../../


# # Build Boost,下面有更新的，舍弃
# wget -O boost_1_64_0.tar.gz https://sourceforge.net/projects/boost/files/boost/1.64.0/boost_1_64_0.tar.gz/download
# tar xzvf boost_1_64_0.tar.gz
# cd boost_1_64_0
# ./bootstrap.sh
# ./b2
# cd ..



#安装opencv,这个项目不需要,到此这个依赖结束
#mkdir -p opencv \
#  && cd opencv \
#  && git clone https://github.com/opencv/opencv.git \
#  &&  \
#  && cd opencv \
#  && git checkout 4.4.0 \
#  && cd ../opencv_contrib \
#  && git checkout 4.4.0 \
#  && cd ../../

# Build opencv，contrib需要对应ceres版本
#sudo apt-get install build-essential pkg-config libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libswscale-dev libtiff5-dev
#mkdir -p opencv/opencv/build \
#	&& mkdir -p opencv/opencv/install\
#	&& cd opencv/opencv/build \
#  && cmake -D CMAKE_BUILD_TYPE=RELEASE \
#    -D CMAKE_INSTALL_PREFIX=/usr/local \
#    -D INSTALL_C_EXAMPLES=ON \
#    -D INSTALL_PYTHON_EXAMPLES=ON \
#    -D OPENCV_GENERATE_PKGCONFIG=ON \
#    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
#    -D BUILD_EXAMPLES=ON .. \
#		-D BUILD_opencv_xfeatures2d=OFF ..
#  && make -j16 \
#  && sudo make install \
#  && cd ../../

#安装g2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 7be4a255de39ae424838899ec800bc3f8b4d8ea6
mkdir build
cd build
cmake ..
make -j10
sudo make install
cd ../../

sudo apt-get install -y libtbb-dev # libboost-all-dev
# Get GTSAM source
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout 4.0.0-alpha2
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_EIGEN_MKL=OFF -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j16
sudo make install
cd ../../


# 删除原来错误的cmake版本：https://blog.csdn.net/qq_42731705/article/details/129379831
# 多版本pcl：https://blog.csdn.net/qq_42731705/article/details/129380907
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout pcl-1.13.0
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
           -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \
           -DCMAKE_INSTALL_PREFIX=/usr ..
make -j16
sudo make install
cd ../../

sudo apt-get update
sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-geodesy
