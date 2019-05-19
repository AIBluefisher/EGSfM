## Global Required(For development: GLOG, GFlags, GTest)
sudo apt-get install libgoogle-glog-dev libgflags-dev libgtest-dev

cd /usr/src/gtest
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo cp libgtest* /usr/lib/
cd ..
sudo rm -rf build

git clone https://github.com/gflags/gflags
cd gflags
mkdir build && cd build
cmake ..
make -j4
sudo make install

## Eigen 3
hg clone https://bitbucket.org/eigen/eigen
mkdir eigen_build && cd eigen_build
cmake . ../eigen
make && sudo make install
cd ..

## Ceres-Solver (Required by both COLMAP and TheiaSfM and OpenMVG)
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make
sudo make install

## Required by openMVG
sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz

## Required by COLMAP
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libcgal-qt5-dev


## Configure and compile COLMAP
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build
cd build
cmake ..
make
sudo make install

#GLFW3 (Optional)
sudo apt-get -y install freeglut3-dev libglew-dev libglfw3-dev

